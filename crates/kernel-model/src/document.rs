// Copyright (c) LMCAD. Licensed under the MIT License.

//! The parametric document: [`Document`] (named parameters + an ordered feature
//! history), its undo/redo wrapper [`DocumentHistory`], and the evaluation that
//! rebuilds geometry from the *current* parameter values.

use std::borrow::Cow;
use std::collections::{BTreeMap, HashMap, HashSet};

use kernel_core::math::{Aabb, Affine3A, DAffine3, DVec3, Mat3, Vec3};
use kernel_core::mesh::{MassProperties, Mesh};
use kernel_core::mesher::Resolution;
use kernel_core::sdf::Sdf;
use kernel_implicit::lattice::{BeamLattice, Pipe};
use kernel_implicit::manifold_dual_contour;
use kernel_implicit::ops::Node;
use kernel_implicit::primitives::{Cuboid, Cylinder, Gyroid, Sphere};
use kernel_implicit::Tpms;
use serde::{Deserialize, Serialize};

use crate::feature::{BooleanOp, Dim, Feature, FeatureId, HoleFit, HoleKind};
use crate::hybrid::{hybrid_boolean, HybridError, HybridOperand, HybridResult};
use crate::meshing::{heal_voxel_size, routed_mesh, watertight_mesh, MeshRoute, RouteReport};
use crate::parts;
use crate::persist;

/// One entry of a [`Document`]'s feature history: the [`Feature`] itself plus the
/// optional human-facing metadata of the user ⇄ AI handoff (BAR.md I5) — a `label`
/// (the short name a person gives the feature, e.g. "mounting boss") and free-form
/// `notes` (design intent, tolerances, reminders).
///
/// The feature is `#[serde(flatten)]`ed, so a record without metadata serializes
/// exactly as the bare feature variant (`{"Box": {…}}` — documents saved before
/// labels existed load unchanged), and a labelled one carries the metadata next to
/// the feature in the file (`{"Box": {…}, "label": "base plate"}`) where a human
/// editing the JSON by hand expects it.
#[derive(Clone, Debug, Serialize, Deserialize)]
struct FeatureRecord {
	/// The geometric feature itself.
	#[serde(flatten)]
	feature: Feature,
	/// Human-readable name (see [`Document::set_label`]).
	#[serde(default, skip_serializing_if = "Option::is_none")]
	label: Option<String>,
	/// Free-form design notes (see [`Document::set_notes`]).
	#[serde(default, skip_serializing_if = "Option::is_none")]
	notes: Option<String>,
}

/// A parametric, re-evaluable model: named parameters plus an ordered feature list.
///
/// The last feature is the document's result unless a different root is set with
/// [`Document::set_root`]. Editing a parameter with [`Document::set_param`] and
/// calling [`Document::evaluate`] / [`Document::mesh`] again produces the updated
/// solid — there is no cached geometry, so updates are always consistent.
///
/// A document is pure data and **persists as JSON** — [`Document::save_json`] /
/// [`Document::load_json`] round-trip it bit-exactly (see [`persist`] for the
/// schema contract), so a modelling session can be resumed from a file.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Document {
	#[serde(serialize_with = "persist::sorted_params")]
	params: HashMap<String, f64>,
	features: Vec<FeatureRecord>,
	root: Option<FeatureId>,
	/// Features toggled off in the rebuild (see [`Document::set_suppressed`]).
	#[serde(serialize_with = "persist::sorted_feature_ids")]
	suppressed: HashSet<FeatureId>,
	/// Named parameter-override sets (see [`Document::add_config`]). `BTreeMap`s
	/// so saves stay byte-stable; skipped when empty, so documents without
	/// configurations serialize exactly as before they existed (and still load in
	/// older kernels).
	#[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
	configs: BTreeMap<String, BTreeMap<String, f64>>,
	/// The active configuration, if any (see [`Document::activate_config`]).
	#[serde(default, skip_serializing_if = "Option::is_none")]
	active_config: Option<String>,
}

impl Document {
	/// An empty document.
	pub fn new() -> Self {
		Self::default()
	}

	/// Set (or insert) a named parameter, returning the previous value if any.
	pub fn set_param(&mut self, name: impl Into<String>, value: f64) -> Option<f64> {
		self.params.insert(name.into(), value)
	}

	/// Get the current value of a named parameter (the **base** value, ignoring
	/// any active configuration — see [`Document::effective_param`]).
	pub fn param(&self, name: &str) -> Option<f64> {
		self.params.get(name).copied()
	}

	/// The value of `name` as evaluation sees it: the active configuration's
	/// override when one applies, the base parameter otherwise.
	pub fn effective_param(&self, name: &str) -> Option<f64> {
		self.active_overrides().and_then(|o| o.get(name).copied()).or_else(|| self.param(name))
	}

	/// All base parameters as `(name, value)` pairs (unordered; collect into a
	/// `BTreeMap` for a sorted view) — the introspection behind parameter
	/// summaries such as a BOM line.
	pub fn params_iter(&self) -> impl Iterator<Item = (&str, f64)> {
		self.params.iter().map(|(k, &v)| (k.as_str(), v))
	}

	/// Add (or replace) a named **configuration**: a set of parameter overrides
	/// that, while the configuration is active, win over the base parameter table
	/// during evaluation — the standard "one model, several variants" mechanism
	/// (a light and a heavy bracket in one `.lmcpart`). Returns the previous
	/// override set under that name, if any. Configurations persist in the saved
	/// document (sorted, byte-stable) and are inert until activated.
	pub fn add_config(
		&mut self,
		name: impl Into<String>,
		overrides: impl IntoIterator<Item = (String, f64)>,
	) -> Option<BTreeMap<String, f64>> {
		self.configs.insert(name.into(), overrides.into_iter().collect())
	}

	/// The override set of configuration `name`, if it exists.
	pub fn config(&self, name: &str) -> Option<&BTreeMap<String, f64>> {
		self.configs.get(name)
	}

	/// All configuration names, sorted.
	pub fn config_names(&self) -> impl Iterator<Item = &str> {
		self.configs.keys().map(String::as_str)
	}

	/// Activate configuration `name`: subsequent evaluations resolve parameters
	/// through its overrides ([`Document::set_param`] keeps editing the base
	/// table, which an override shadows until deactivation). Returns `false` —
	/// and changes nothing — when no such configuration exists, so a typo cannot
	/// silently evaluate the base variant.
	pub fn activate_config(&mut self, name: &str) -> bool {
		if self.configs.contains_key(name) {
			self.active_config = Some(name.to_string());
			true
		} else {
			false
		}
	}

	/// Deactivate any active configuration (back to the base parameter table).
	pub fn deactivate_config(&mut self) {
		self.active_config = None;
	}

	/// The active configuration's name, if one is active.
	pub fn active_config(&self) -> Option<&str> {
		self.active_config.as_deref()
	}

	/// The active configuration's overrides, if an active name resolves. A
	/// hand-edited `active_config` naming a missing configuration resolves to no
	/// overrides (the base variant) — the name is kept so saving preserves it.
	fn active_overrides(&self) -> Option<&BTreeMap<String, f64>> {
		self.active_config.as_deref().and_then(|n| self.configs.get(n))
	}

	/// The parameter table evaluation resolves against: the base table, with the
	/// active configuration's overrides applied on top (borrowed when no
	/// configuration is active, so the common path stays allocation-free).
	fn effective_params(&self) -> Cow<'_, HashMap<String, f64>> {
		match self.active_overrides() {
			None => Cow::Borrowed(&self.params),
			Some(overrides) => {
				let mut merged = self.params.clone();
				for (k, v) in overrides {
					merged.insert(k.clone(), *v);
				}
				Cow::Owned(merged)
			}
		}
	}

	/// Append a feature, returning its [`FeatureId`].
	///
	/// The newly added feature becomes the document root unless one was pinned
	/// with [`Document::set_root`].
	pub fn add(&mut self, feature: Feature) -> FeatureId {
		let id = FeatureId(self.features.len());
		self.features.push(FeatureRecord { feature, label: None, notes: None });
		id
	}

	/// Insert a feature at `index` (clamped to the history's length), shifting
	/// the features at and after that position one step later — the parametric
	/// "drag a feature earlier into the history" edit. Every [`FeatureId`]
	/// reference in later features, the pinned root, and the suppression set are
	/// remapped, so the document rebuilds exactly as before with the new feature
	/// available at `index` (labels and notes travel with their features).
	/// Returns the new feature's id, `FeatureId(index)`. The inserted feature may
	/// reference only features before `index` (ids are history positions).
	pub fn insert_feature_at(&mut self, index: usize, feature: Feature) -> FeatureId {
		let index = index.min(self.features.len());
		let shift = |id: FeatureId| if id.0 >= index { FeatureId(id.0 + 1) } else { id };
		for record in self.features.iter_mut().skip(index) {
			remap_feature_refs(&mut record.feature, shift);
		}
		self.features.insert(index, FeatureRecord { feature, label: None, notes: None });
		if let Some(root) = self.root.as_mut() {
			*root = shift(*root);
		}
		self.suppressed = std::mem::take(&mut self.suppressed).into_iter().map(shift).collect();
		FeatureId(index)
	}

	/// Pin a specific feature as the document's result.
	pub fn set_root(&mut self, id: FeatureId) {
		self.root = Some(id);
	}

	/// Suppress or un-suppress a feature — the standard parametric-edit toggle that
	/// switches a feature off in the rebuild without deleting it (so an AI can compare
	/// design variants). A suppressed **modifier** feature (one with a single upstream
	/// input: fillet, chamfer, shell, transform, linear/circular pattern, mirror) is
	/// replaced by that input on the next [`Document::evaluate`] / [`evaluate_brep`].
	/// Suppress is a no-op for **generative** features (primitives, booleans, smooth
	/// booleans, sketches, lattices), which have no single input to fall back to.
	pub fn set_suppressed(&mut self, id: FeatureId, suppressed: bool) {
		if suppressed {
			self.suppressed.insert(id);
		} else {
			self.suppressed.remove(&id);
		}
	}

	/// Whether `id` is currently suppressed.
	pub fn is_suppressed(&self, id: FeatureId) -> bool {
		self.suppressed.contains(&id)
	}

	/// Set (or replace) the human-readable **label** of feature `id` — the short
	/// name a person gives a feature ("mounting boss", "M5 bore"), persisted in
	/// the saved JSON next to the feature so a hand-editing user and an AI session
	/// share the same vocabulary (BAR.md I5). Purely descriptive: labels never
	/// affect evaluation. No-op when `id` names no feature; returns the previous
	/// label, if any.
	pub fn set_label(&mut self, id: FeatureId, label: impl Into<String>) -> Option<String> {
		self.features.get_mut(id.0).and_then(|record| record.label.replace(label.into()))
	}

	/// The label of feature `id`, if one was set (see [`Document::set_label`]).
	pub fn label(&self, id: FeatureId) -> Option<&str> {
		self.features.get(id.0).and_then(|record| record.label.as_deref())
	}

	/// Set (or replace) the free-form **notes** of feature `id` — design intent,
	/// tolerances, reminders; the long-form sibling of [`Document::set_label`]
	/// with the same persistence and no-effect-on-evaluation semantics. No-op when
	/// `id` names no feature; returns the previous notes, if any.
	pub fn set_notes(&mut self, id: FeatureId, notes: impl Into<String>) -> Option<String> {
		self.features.get_mut(id.0).and_then(|record| record.notes.replace(notes.into()))
	}

	/// The notes of feature `id`, if any were set (see [`Document::set_notes`]).
	pub fn notes(&self, id: FeatureId) -> Option<&str> {
		self.features.get(id.0).and_then(|record| record.notes.as_deref())
	}

	/// The feature currently acting as the result, if the document is non-empty.
	pub fn root(&self) -> Option<FeatureId> {
		self.root.or_else(|| if self.features.is_empty() { None } else { Some(FeatureId(self.features.len() - 1)) })
	}

	/// Rebuild the CSG [`Node`] from the *current* parameter values.
	///
	/// Returns `None` for an empty document or one whose root references a
	/// missing / cyclic feature. The tree is built fresh every call, so it always
	/// reflects the latest [`Document::set_param`] edits.
	pub fn evaluate(&self) -> Option<Node> {
		self.evaluate_to(self.root()?)
	}

	/// **Rollback** evaluation: rebuild the CSG [`Node`] as the model stood at
	/// feature `id` — the prefix of the history up to and including it (the
	/// feature-tree rollback bar every parametric modeller has). Suppression and
	/// the active configuration apply as usual; the pinned root is ignored.
	/// `None` for an unknown id or a prefix with no implicit form.
	pub fn evaluate_to(&self, id: FeatureId) -> Option<Node> {
		if id.0 >= self.features.len() {
			return None;
		}
		// A feature DAG with shared sub-features expands into a tree, so a diamond
		// chain would re-evaluate exponentially. The SDF `Node` cannot share subtrees
		// (its leaves are boxed), so we cap the total expansion to a generous multiple
		// of the feature count and bail rather than hang. (A truly shared
		// representation would need `Arc`-backed nodes — tracked as follow-up.)
		let mut budget = self.features.len().saturating_mul(64).max(1024);
		let params = self.effective_params();
		self.build(id, &mut Vec::new(), &mut budget, &params)
	}

	/// Recursively build the node for `id`, using `stack` to reject cycles,
	/// `budget` to bound DAG expansion, and `params` (the configuration-resolved
	/// table) for every [`Dim`].
	fn build(&self, id: FeatureId, stack: &mut Vec<FeatureId>, budget: &mut usize, params: &HashMap<String, f64>) -> Option<Node> {
		if *budget == 0 {
			return None; // expansion budget exhausted (pathological shared-feature DAG)
		}
		*budget -= 1;
		let feature = &self.features.get(id.0)?.feature;
		if stack.contains(&id) {
			return None; // cyclic reference: bail rather than recurse forever
		}
		stack.push(id);
		// A suppressed modifier feature is replaced by its upstream input.
		if self.suppressed.contains(&id) {
			if let Some(inp) = primary_input(feature) {
				let node = self.build(inp, stack, budget, params);
				stack.pop();
				return node;
			}
		}
		let node = match feature {
			Feature::Box { center, size } => {
				let c = resolve_vec3(params, center);
				let half = resolve_vec3(params, size) * 0.5;
				// Guard against negative/zero dimensions producing an inverted box.
				let half = half.abs();
				Some(Node::primitive(Cuboid::new(c, half)))
			}
			Feature::Sphere { center, radius } => {
				let c = resolve_vec3(params, center);
				let r = radius.resolve(params).max(0.0) as f32;
				Some(Node::primitive(Sphere::new(c, r)))
			}
			Feature::Cylinder { center, radius, height } => {
				let c = resolve_vec3(params, center);
				let r = radius.resolve(params).max(0.0) as f32;
				let h = (height.resolve(params).max(0.0) as f32) * 0.5;
				let a = c - Vec3::new(0.0, 0.0, h);
				let b = c + Vec3::new(0.0, 0.0, h);
				Some(Node::primitive(Cylinder::new(a, b, r)))
			}
			Feature::Boolean { op, a, b } => {
				let na = self.build(*a, stack, budget, params)?;
				let nb = self.build(*b, stack, budget, params)?;
				Some(match op {
					BooleanOp::Union => na.union(nb),
					BooleanOp::Difference => na.difference(nb),
					BooleanOp::Intersection => na.intersection(nb),
				})
			}
			Feature::Gyroid { center, size, scale, thickness } => {
				let c = resolve_vec3(params, center);
				let half = (resolve_vec3(params, size) * 0.5).abs();
				let sc = scale.resolve(params).max(0.0) as f32;
				let th = thickness.resolve(params).max(0.0) as f32;
				let region = Aabb::from_center_half_extent(c, half);
				// The TPMS field is bounded by intersecting it with its box, giving a
				// lattice block; intersect that with a part for true infill.
				let lattice = Node::primitive(Gyroid::new(region, sc, th));
				Some(lattice.intersection(Node::primitive(Cuboid::new(c, half))))
			}
			Feature::SmoothUnion { a, b, blend } => {
				let na = self.build(*a, stack, budget, params)?;
				let nb = self.build(*b, stack, budget, params)?;
				let k = blend.resolve(params).max(0.0) as f32;
				Some(na.smooth_union(nb, k))
			}
			Feature::SmoothDifference { a, b, blend } => {
				let na = self.build(*a, stack, budget, params)?;
				let nb = self.build(*b, stack, budget, params)?;
				let k = blend.resolve(params).max(0.0) as f32;
				Some(na.smooth_difference(nb, k))
			}
			Feature::SmoothIntersection { a, b, blend } => {
				let na = self.build(*a, stack, budget, params)?;
				let nb = self.build(*b, stack, budget, params)?;
				let k = blend.resolve(params).max(0.0) as f32;
				Some(na.smooth_intersection(nb, k))
			}
			Feature::Transform { input, xform } => {
				let n = self.build(*input, stack, budget, params)?;
				Some(n.transform(*xform))
			}
			// The implicit/voxel preview has no edge topology to name, so it cannot
			// apply a B-rep edge fillet/chamfer — it returns the input unmodified. The
			// exact result is produced by `evaluate_brep`.
			Feature::Fillet { input, .. } | Feature::Chamfer { input, .. } => self.build(*input, stack, budget, params),
			// An extruded sketch is a B-rep-only feature: the implicit preview has no
			// 2D-profile primitive to represent it, so it does not appear on this path.
			Feature::ExtrudeSketch { .. } => None,
			Feature::FilletedCylinder { .. } | Feature::ChamferedCylinder { .. } => None, // B-rep-only
			Feature::LinearPattern { input, count, step } => {
				let s = resolve_vec3(params, step);
				let mut acc = self.build(*input, stack, budget, params)?;
				for k in 1..(*count).max(1) {
					let copy = self.build(*input, stack, budget, params)?.transform(Affine3A::from_translation(s * k as f32));
					acc = acc.union(copy);
				}
				Some(acc)
			}
			Feature::Mirror { input, plane_point, plane_normal } => {
				let base = self.build(*input, stack, budget, params)?;
				let copy = self
					.build(*input, stack, budget, params)?
					.transform(reflection_affine(resolve_vec3(params, plane_point), resolve_vec3(params, plane_normal)));
				Some(base.union(copy))
			}
			Feature::CircularPattern { input, count, axis_point, axis_dir, angle } => {
				let p = resolve_vec3(params, axis_point);
				let axis = resolve_vec3(params, axis_dir).normalize_or_zero();
				let step = angle.resolve(params) as f32;
				let mut acc = self.build(*input, stack, budget, params)?;
				if axis.length_squared() >= 0.5 {
					for k in 1..(*count).max(1) {
						let rot = Affine3A::from_translation(p)
							* Affine3A::from_axis_angle(axis, step * k as f32)
							* Affine3A::from_translation(-p);
						acc = acc.union(self.build(*input, stack, budget, params)?.transform(rot));
					}
				}
				Some(acc)
			}
			Feature::Shell { input, thickness } => {
				// Voxel-half shell: keep the outer surface and subtract an inward-offset
				// copy so a wall of `thickness` remains, outer dimensions preserved. Built
				// twice (like Mirror) since the SDF tree cannot share boxed subnodes.
				let w = thickness.resolve(params).max(0.0) as f32;
				let outer = self.build(*input, stack, budget, params)?;
				let inner = self.build(*input, stack, budget, params)?.offset(-w);
				Some(outer.difference(inner))
			}
			Feature::GyroidLattice { region, scale, thickness, grade } => {
				let (c, half) = resolve_region(params, region);
				let sc = scale.resolve(params).max(0.0) as f32;
				let th = thickness.resolve(params).max(0.0) as f32;
				let lattice = Node::primitive(Gyroid::new(Aabb::from_center_half_extent(c, half), sc, th));
				// The grade inflates the TPMS walls BEFORE the region clamp, so the box
				// boundary stays put while the sheets thicken/thin along the law. The
				// closure captures the RESOLVED constants — a parameter edit re-resolves
				// them on the next evaluate, and the same document always compiles the
				// same field (deterministic, R5).
				let graded = match grade {
					None => lattice,
					Some(g) => {
						let axis = resolve_vec3(params, &g.axis);
						let rate = g.per_unit.resolve(params) as f32;
						let offset = g.offset.resolve(params) as f32;
						let max_abs = g.max_abs.resolve(params).max(0.0) as f32;
						lattice.offset_by(std::sync::Arc::new(move |p: Vec3| offset + rate * axis.dot(p)), max_abs)
					}
				};
				Some(graded.intersection(Node::primitive(Cuboid::new(c, half))))
			}
			Feature::Tpms { region, kind, cell, sheet, level } => {
				let (c, half) = resolve_region(params, region);
				let region_box = Aabb::from_center_half_extent(c, half);
				let cell_mm = cell.resolve(params);
				let lv = level.as_ref().map(|d| d.resolve(params)).unwrap_or(0.0);
				// Fail-loud guards (None, never a panic): positive finite cell, finite
				// region/level, and a positive wall half-thickness in sheet mode.
				let inputs_sound =
					cell_mm > 0.0 && cell_mm.is_finite() && c.is_finite() && half.is_finite() && lv.is_finite() && (!*sheet || lv > 0.0);
				if !inputs_sound {
					None
				} else {
					let field = if *sheet {
						Tpms::sheet(region_box, kind.kind(), cell_mm as f32, lv as f32)
					} else {
						Tpms::network(region_box, kind.kind(), cell_mm as f32, lv as f32)
					};
					// A raw TPMS is an OPEN labyrinth (the region box cuts its tubes) —
					// clamp with the region so the block is a closed solid, exactly like
					// `Feature::Gyroid` / `Feature::GyroidLattice`.
					Some(Node::primitive_bound(field).intersection(Node::primitive(Cuboid::new(c, half))))
				}
			}
			Feature::BeamLatticeFill { region, cell, cell_size, radius } => {
				let (c, half) = resolve_region(params, region);
				let cs = cell_size.resolve(params);
				let r = radius.resolve(params);
				// Fail-loud guards (None, never a panicking `from_cells`): positive
				// finite strut dimensions, a finite region and a bounded cell count.
				if !(cs > 0.0 && cs.is_finite() && r > 0.0 && r.is_finite() && c.is_finite() && half.is_finite()) {
					None
				} else {
					let region_box = Aabb::from_center_half_extent(c, half);
					// Same per-axis count as `from_cells`: floor(size/cell), at least 1.
					let n = |s: f32| ((s as f64 / cs).floor() as usize).max(1);
					let size = region_box.size();
					let cells = n(size.x).saturating_mul(n(size.y)).saturating_mul(n(size.z));
					(cells <= LATTICE_FILL_MAX_CELLS)
						.then(|| Node::primitive(BeamLattice::from_cells(region_box, cell.to_implicit(), cs as f32, r as f32)))
				}
			}
			Feature::PipeFeat { path, radii } => {
				// Fail-loud guards mirroring `Pipe::new`'s asserted contract.
				if path.len() < 2 || path.len() != radii.len() {
					None
				} else {
					let pts: Vec<Vec3> = path.iter().map(|p| resolve_vec3(params, p)).collect();
					let rs: Vec<f32> = radii.iter().map(|r| r.resolve(params) as f32).collect();
					(pts.iter().all(|p| p.is_finite()) && rs.iter().all(|r| *r > 0.0 && r.is_finite()))
						.then(|| Node::primitive(Pipe::new(pts, rs)))
				}
			}
			Feature::HybridFuse { brep, field, op, .. } => {
				// The voxel twin of the fuse (the exact result lives on `build_brep`):
				// the exact operand is built, tessellated, and lifted into its
				// winding-number field (`MeshSdf`, the `Instance::from_mesh` move), then
				// combined with the field operand by min/max — the same construction as
				// the hybrid's healed route, so this path stays meshable and watertight
				// even when the exact stitch is refused. The shared stack/budget reject
				// a hand-edited cyclic fuse across both halves.
				let solid = self.build_brep(*brep, stack, budget, params)?;
				let node = self.build(*field, stack, budget, params)?;
				let lifted = Node::primitive(kernel_implicit::MeshSdf::new(&kernel_brep::tessellate_default(&solid)));
				Some(match op {
					BooleanOp::Union => lifted.union(node),
					BooleanOp::Difference => lifted.difference(node),
					BooleanOp::Intersection => lifted.intersection(node),
				})
			}
			// Hole-wizard cuts, lofts/sweeps and catalog parts are B-rep-only: their
			// table-driven tool geometry / skinned topology has no implicit twin, so
			// they are absent on this path (the mirror of `Feature::ExtrudeSketch`) —
			// a preview must not silently show a part without its holes.
			Feature::Hole { .. }
			| Feature::LoftSolid { .. }
			| Feature::SweepSolid { .. }
			| Feature::Revolve { .. }
			| Feature::CatalogPart { .. } => None,
			// Rim fillets and the standard grooves / insert bosses are small local
			// B-rep modifications; like `Feature::Fillet`, the implicit preview passes
			// the input through unmodified and `evaluate_brep` carries the exact result.
			Feature::CircularRimFillet { input, .. }
			| Feature::ORingGroove { input, .. }
			| Feature::CirclipGroove { input, .. }
			| Feature::HeatsetBoss { input, .. } => self.build(*input, stack, budget, params),
		};
		stack.pop();
		node
	}

	/// Build the document as an **exact B-rep** [`kernel_brep::Solid`] rather than an
	/// implicit field — mirrors [`Document::evaluate`] but uses the B-rep primitives
	/// and booleans, so the result carries persistent face provenance
	/// ([`kernel_brep::FaceName`]). An agent can therefore select a result face by a
	/// name that survives a parameter edit (`face_name` / `faces_named`), the
	/// foundation of parametric feature references. `None` for an empty/cyclic document.
	///
	/// Curved primitives are faceted (cylinder/sphere use a fixed segment count) since
	/// the B-rep boolean operates on planar faces.
	pub fn evaluate_brep(&self) -> Option<kernel_brep::Solid> {
		self.evaluate_brep_to(self.root()?)
	}

	/// **Rollback** evaluation on the exact half: build the B-rep as the model
	/// stood at feature `id` — the prefix of the history up to and including it
	/// (the B-rep counterpart of [`Document::evaluate_to`]). Suppression and the
	/// active configuration apply; the pinned root is ignored. `None` for an
	/// unknown id or a prefix with no B-rep form.
	pub fn evaluate_brep_to(&self, id: FeatureId) -> Option<kernel_brep::Solid> {
		if id.0 >= self.features.len() {
			return None;
		}
		let mut budget = self.features.len().saturating_mul(64).max(1024);
		let params = self.effective_params();
		self.build_brep(id, &mut Vec::new(), &mut budget, &params)
	}

	/// Exact rigid-body [`MassProperties`] (volume, centre of mass, inertia at unit density) of
	/// this document's evaluated B-rep — the mass of the parametric part in one call, without
	/// the caller reaching for [`evaluate_brep`](Self::evaluate_brep). Re-evaluated each call,
	/// so it tracks parameter edits. `None` when the document has no B-rep result (e.g. an
	/// organic / implicit-only model).
	pub fn mass_properties(&self) -> Option<MassProperties> {
		self.evaluate_brep().map(|s| kernel_brep::mass_properties(&s))
	}

	/// Recursive B-rep counterpart of [`Document::build`].
	fn build_brep(
		&self,
		id: FeatureId,
		stack: &mut Vec<FeatureId>,
		budget: &mut usize,
		params: &HashMap<String, f64>,
	) -> Option<kernel_brep::Solid> {
		if *budget == 0 {
			return None;
		}
		*budget -= 1;
		let feature = &self.features.get(id.0)?.feature;
		if stack.contains(&id) {
			return None;
		}
		stack.push(id);
		// A suppressed modifier feature is replaced by its upstream input.
		if self.suppressed.contains(&id) {
			if let Some(inp) = primary_input(feature) {
				let solid = self.build_brep(inp, stack, budget, params);
				stack.pop();
				return solid;
			}
		}
		let dv = |d: &[Dim; 3]| DVec3::new(d[0].resolve(params), d[1].resolve(params), d[2].resolve(params));
		let solid = match feature {
			Feature::Box { center, size } => {
				let c = dv(center);
				let half = dv(size).abs() * 0.5;
				Some(kernel_brep::cuboid(c - half, c + half))
			}
			Feature::Sphere { center, radius } => {
				let r = radius.resolve(params).max(0.0);
				Some(kernel_brep::sphere(dv(center), r, 32, 16))
			}
			Feature::Cylinder { center, radius, height } => {
				let c = dv(center);
				let r = radius.resolve(params).max(0.0);
				let h = height.resolve(params).max(0.0);
				Some(kernel_brep::cylinder(c - DVec3::new(0.0, 0.0, h * 0.5), DVec3::Z, r, h, 32))
			}
			Feature::FilletedCylinder { radius, height, fillet } => {
				let r = radius.resolve(params).max(0.0);
				let h = height.resolve(params).max(0.0);
				let f = fillet.resolve(params).max(0.0);
				Some(kernel_brep::filleted_cylinder(r, h, f, 48, 8))
			}
			Feature::ChamferedCylinder { radius, height, chamfer } => {
				let r = radius.resolve(params).max(0.0);
				let h = height.resolve(params).max(0.0);
				let c = chamfer.resolve(params).max(0.0);
				Some(kernel_brep::chamfered_cylinder(r, h, c, 48))
			}
			Feature::Boolean { op, a, b } => {
				let sa = self.build_brep(*a, stack, budget, params)?;
				let sb = self.build_brep(*b, stack, budget, params)?;
				Some(match op {
					BooleanOp::Union => kernel_brep::union(&sa, &sb),
					BooleanOp::Difference => kernel_brep::difference(&sa, &sb),
					BooleanOp::Intersection => kernel_brep::intersection(&sa, &sb),
				})
			}
			// Smooth/filleted booleans and the gyroid TPMS lattice are voxel-half organic
			// ops (smin/smax on the SDF, or a TPMS field); the exact B-rep half has no
			// analytic representation, so they are absent here (the mirror of
			// `Feature::Shell`). Mesh them via `evaluate` / `mesh`.
			Feature::SmoothUnion { .. }
			| Feature::SmoothDifference { .. }
			| Feature::SmoothIntersection { .. }
			| Feature::Gyroid { .. } => None,
			Feature::Transform { input, xform } => {
				let s = self.build_brep(*input, stack, budget, params)?;
				let m = xform.matrix3;
				let daff = DAffine3::from_cols(m.x_axis.as_dvec3(), m.y_axis.as_dvec3(), m.z_axis.as_dvec3(), xform.translation.as_dvec3());
				Some(s.transformed(daff))
			}
			Feature::Fillet { input, edge, radius, near } => {
				let s = self.build_brep(*input, stack, budget, params)?;
				let r = radius.resolve(params);
				// The edge name re-resolves against the freshly-rebuilt input, so the
				// fillet re-attaches after an upstream edit. With a `near` witness a name
				// that split into fragments is disambiguated to the nearest one; without,
				// an unresolved/ambiguous edge makes the document fail to evaluate rather
				// than silently return an unrounded solid.
				match near {
					Some(w) => kernel_brep::fillet_edge_near(&s, *edge, r, dv(w)).ok(),
					None => kernel_brep::fillet_edge(&s, *edge, r).ok(),
				}
			}
			Feature::Chamfer { input, edge, radius, near } => {
				let s = self.build_brep(*input, stack, budget, params)?;
				let r = radius.resolve(params);
				match near {
					Some(w) => kernel_brep::chamfer_edge_near(&s, *edge, r, dv(w)).ok(),
					None => kernel_brep::chamfer_edge(&s, *edge, r).ok(),
				}
			}
			Feature::ExtrudeSketch { sketch, height, dims, draft } => {
				// Apply the parametric dimension overrides, then solve the constraints on
				// every rebuild (cheap, idempotent) so the profile reflects the current
				// parameters, and extrude by the parameter-resolved height with the
				// parameter-resolved draft (0 ⇒ a plain prism, full hole support).
				let h = height.resolve(params);
				let a = draft.resolve(params);
				let mut sk = sketch.clone();
				for (index, dim) in dims {
					sk.set_distance(*index, dim.resolve(params));
				}
				sk.solve();
				sk.extrude_tapered(h, a).ok()
			}
			Feature::LinearPattern { input, count, step } => {
				let base = self.build_brep(*input, stack, budget, params)?;
				let s = DVec3::new(step[0].resolve(params), step[1].resolve(params), step[2].resolve(params));
				// If adjacent copies are AABB-disjoint, merge their topology directly (exact, and
				// avoids the chained curved-boolean corruption that self-intersects); otherwise the
				// copies touch/overlap and must be fused with a real boolean union.
				let merge = *count < 2 || aabb_disjoint(&base, &base.transformed(DAffine3::from_translation(s)));
				let mut acc = base.clone();
				for k in 1..(*count).max(1) {
					let copy = base.transformed(DAffine3::from_translation(s * k as f64));
					acc = if merge { acc.disjoint_union(&copy) } else { kernel_brep::union(&acc, &copy) };
				}
				Some(acc)
			}
			Feature::Mirror { input, plane_point, plane_normal } => {
				let base = self.build_brep(*input, stack, budget, params)?;
				let mirror = base.mirrored(dv(plane_point), dv(plane_normal));
				// A mirror plane that doesn't cut the part leaves base and its reflection disjoint →
				// merge their topology exactly (avoids the curved-boolean corruption); a cutting plane
				// makes them overlap on the seam → fuse with a real boolean union.
				let combined =
					if aabb_disjoint(&base, &mirror) { base.disjoint_union(&mirror) } else { kernel_brep::union(&base, &mirror) };
				Some(combined)
			}
			Feature::CircularPattern { input, count, axis_point, axis_dir, angle } => {
				let base = self.build_brep(*input, stack, budget, params)?;
				let p = dv(axis_point);
				let axis = dv(axis_dir).normalize_or_zero();
				let step = angle.resolve(params);
				let mut acc = base.clone();
				if axis.length_squared() >= 0.5 {
					let rot1 = DAffine3::from_translation(p) * DAffine3::from_axis_angle(axis, step) * DAffine3::from_translation(-p);
					// Disjoint copies (a typical bolt circle) merge by exact topology; overlapping
					// copies fuse with a real boolean union (see LinearPattern for the rationale).
					let merge = *count < 2 || aabb_disjoint(&base, &base.transformed(rot1));
					for k in 1..(*count).max(1) {
						let rot = DAffine3::from_translation(p)
							* DAffine3::from_axis_angle(axis, step * k as f64)
							* DAffine3::from_translation(-p);
						let copy = base.transformed(rot);
						acc = if merge { acc.disjoint_union(&copy) } else { kernel_brep::union(&acc, &copy) };
					}
				}
				Some(acc)
			}
			// A shell is a voxel-half op (inward offset + CSG difference); the exact
			// B-rep half has no general face-offset yet, so it is absent from this path
			// (the mirror of `ExtrudeSketch`, which is B-rep-only and returns `None` on
			// the implicit path). Mesh the hollowed solid via `evaluate` / `mesh`.
			Feature::Shell { .. } => None,
			Feature::Hole { input, kind, m_or_d, at, axis, fit, depth } => {
				let s = self.build_brep(*input, stack, budget, params)?;
				let at = dv(at);
				let axis = dv(axis);
				let size = m_or_d.resolve(params);
				match kind {
					// Drill / tap pilot: no fit series applies; depth `None` bores
					// through the part's whole extent along the axis from `at`.
					HoleKind::Drill | HoleKind::Tap => {
						if fit.is_some() {
							return None; // a fit series on a drill/tap is a contradiction — loud
						}
						let hole_depth = match depth {
							Some(d) => kernel_brep::HoleDepth::Blind(d.resolve(params)),
							None => kernel_brep::HoleDepth::Through(through_length(&s, at, axis)?),
						};
						match kind {
							HoleKind::Drill => kernel_brep::drill(&s, at, axis, size, hole_depth, None).ok(),
							_ => kernel_brep::tap_drill_hole(&s, at, axis, size, hole_depth, None).ok(),
						}
					}
					// The fastener seats are through cuts by definition; a depth here
					// would be silently ignored, so it fails loudly instead.
					HoleKind::Clearance | HoleKind::Counterbore | HoleKind::Countersink => {
						if depth.is_some() {
							return None;
						}
						let fit = fit.unwrap_or(HoleFit::Medium).to_brep();
						match kind {
							HoleKind::Clearance => kernel_brep::clearance_hole(&s, at, axis, size, fit, None).ok(),
							HoleKind::Counterbore => kernel_brep::counterbore_hole(&s, at, axis, size, fit, None).ok(),
							_ => kernel_brep::countersink_hole(&s, at, axis, size, fit, None).ok(),
						}
					}
				}
			}
			Feature::CircularRimFillet { input, near, radius, concave } => {
				let s = self.build_brep(*input, stack, budget, params)?;
				let witness = dv(near);
				let r = radius.resolve(params);
				// Out-of-scope rims return None, so the document fails to evaluate
				// rather than silently dropping the round (same contract as Fillet).
				if *concave {
					kernel_brep::fillet_circular_rim_concave(&s, witness, r, RIM_FILLET_ARC_SEGMENTS)
				} else {
					kernel_brep::fillet_circular_rim(&s, witness, r, RIM_FILLET_ARC_SEGMENTS)
				}
			}
			Feature::LoftSolid { sections } => {
				let sections: Vec<Vec<DVec3>> = sections.iter().map(|loop_| loop_.iter().map(dv).collect()).collect();
				kernel_brep::loft_solid(&sections)
			}
			Feature::SweepSolid { profile, path } => {
				let profile: Vec<DVec3> = profile.iter().map(dv).collect();
				let path: Vec<DVec3> = path.iter().map(dv).collect();
				kernel_brep::sweep_solid(&profile, &path)
			}
			Feature::Revolve { profile, segments } => {
				let profile: Vec<kernel_core::math::DVec2> =
					profile.iter().map(|p| kernel_core::math::DVec2::new(p[0].resolve(params), p[1].resolve(params))).collect();
				let segs = if *segments == 0 { 64 } else { *segments };
				Some(kernel_brep::revolve(&profile, segs))
			}
			Feature::CatalogPart { part } => part.build(params),
			Feature::ORingGroove { input, at, axis, dash } => {
				let s = self.build_brep(*input, stack, budget, params)?;
				parts::o_ring_groove(&s, dv(at), dv(axis), *dash)
			}
			Feature::CirclipGroove { input, at, axis, d, internal } => {
				let s = self.build_brep(*input, stack, budget, params)?;
				let d = d.resolve(params);
				if *internal {
					parts::circlip_groove_internal(&s, dv(at), dv(axis), d)
				} else {
					parts::circlip_groove_external(&s, dv(at), dv(axis), d)
				}
			}
			Feature::HeatsetBoss { input, at, axis, m } => {
				let s = self.build_brep(*input, stack, budget, params)?;
				parts::heatset_insert_boss(&s, dv(at), dv(axis), m.resolve(params))
			}
			// The TPMS / beam-lattice / pipe fills are voxel-half organic bodies: the
			// exact half has no analytic twin for them, so they are absent from this
			// path (the mirror of `Feature::Shell`). Mesh them via `evaluate` / `mesh`,
			// or fuse them onto an exact part with `Feature::HybridFuse`.
			Feature::GyroidLattice { .. } | Feature::Tpms { .. } | Feature::BeamLatticeFill { .. } | Feature::PipeFeat { .. } => None,
			Feature::HybridFuse { brep, field, op, voxel } => {
				// The cross-representation boolean: exact operand as a Solid, field
				// operand as a Node meshed at `voxel` (see `hybrid_boolean`). On the
				// EXACT-STITCH route the stitched partial-credit Solid (untouched faces
				// verbatim, provenance-tagged seam) feeds downstream B-rep features. On
				// the HEALED route — or a HybridError — this is None: a mesh-only result
				// honestly cannot chain into exact features. The watertight mesh remains
				// reachable via `Document::mesh` / `export_mesh` (which then states the
				// heal), and the route + reason + report via `hybrid_fuse_result`.
				let a = self.build_brep(*brep, stack, budget, params)?;
				let b = self.build(*field, stack, budget, params)?;
				let v = voxel.resolve(params) as f32;
				match hybrid_boolean(&a, HybridOperand::Node(&b), *op, v) {
					Ok(out) => out.solid,
					Err(_) => None,
				}
			}
		};
		stack.pop();
		solid
	}

	/// Evaluate and mesh the document at the given `resolution`.
	///
	/// Meshed with Manifold Dual Contouring, so the result is a closed **2-manifold**
	/// with sharp edges preserved — a `Difference` feature's concave crease comes out
	/// watertight rather than with the non-manifold edges plain Surface Nets leaves
	/// there. Returns an empty [`Mesh`] for an empty / invalid document.
	pub fn mesh(&self, resolution: impl Into<Resolution>) -> Mesh {
		match self.evaluate() {
			Some(node) => {
				let bounds = node.bounds();
				manifold_dual_contour(&node, bounds, resolution)
			}
			None => Mesh::new(),
		}
	}

	/// Mesh the document's **exact B-rep** result into a watertight mesh via the
	/// hybrid heal ([`watertight_mesh`]) at `voxel_size`. Returns an empty mesh if
	/// the document has no valid B-rep.
	///
	/// This is the B-rep counterpart of [`Document::mesh`] (which meshes the
	/// implicit/voxel tree directly): it builds the exact B-rep, then recovers a
	/// printable watertight mesh through the voxel half — so an AI gets a sound
	/// mesh of a parametric solid even when its exact tessellation has curved-face
	/// cracks.
	pub fn watertight_brep_mesh(&self, voxel_size: f32) -> Mesh {
		match self.evaluate_brep() {
			Some(solid) => watertight_mesh(&solid, voxel_size),
			None => Mesh::new(),
		}
	}

	/// Export this document as a mesh at chord tolerance `tol` (mm) through the
	/// kernel's **routing policy**, returning the mesh together with the
	/// [`RouteReport`] saying which path produced it and why — the one call that
	/// centralizes the exact-else-heal decision so callers stop hand-rolling it:
	///
	/// - a document with an exact B-rep routes through [`routed_mesh`]
	///   (self-intersection check → exact adaptive tessellation when watertight →
	///   voxel heal otherwise);
	/// - a document with **no** B-rep form (voxel-half features such as
	///   [`Feature::Shell`] / smooth booleans / [`Feature::Gyroid`]) is meshed on
	///   the SDF half and reported [`MeshRoute::Healed`];
	/// - an empty / invalid document returns an empty mesh (`tris == 0`,
	///   `watertight == false`) with the reason in [`RouteReport::why`].
	pub fn export_mesh(&self, tol: f64) -> (Mesh, RouteReport) {
		if let Some(solid) = self.evaluate_brep() {
			return routed_mesh(&solid, tol);
		}
		let mesh = self.mesh(Resolution::VoxelSize(heal_voxel_size(tol)));
		let report = if mesh.is_empty() {
			RouteReport::for_mesh(&mesh, MeshRoute::Healed, "empty or invalid document: no geometry to export")
		} else {
			RouteReport::for_mesh(
				&mesh,
				MeshRoute::Healed,
				"no exact B-rep for this document (voxel-half features); meshed on the SDF half",
			)
		};
		(mesh, report)
	}

	/// Re-run the [`Feature::HybridFuse`] at `id` and return its **full routed
	/// result**: the verified-watertight mesh, the [`HybridRoute`] taken (with
	/// the healed route's stated reason), the measured per-face [`HybridReport`],
	/// and — on the exact route — the stitched solid. This is the retrieval
	/// mechanism for a fuse's route: a [`Document`] is pure persisted data, so
	/// nothing is cached; the fuse is **recomputed at call time**, which is sound
	/// because the kernel rebuild is deterministic (R5) — the same document
	/// yields the same route, report and mesh bits as the
	/// [`evaluate_brep`](Self::evaluate_brep) that built it.
	///
	/// `None` when `id` does not name a `HybridFuse` or an operand fails to
	/// evaluate; `Some(Err(_))` when neither hybrid route produced a watertight
	/// result ([`HybridError`], loud); `Some(Ok(_))` otherwise — on the healed
	/// route the result's `solid` is `None` and `mesh` is the watertight voxel
	/// fuse. Suppression and the active configuration apply as usual.
	pub fn hybrid_fuse_result(&self, id: FeatureId) -> Option<Result<HybridResult, HybridError>> {
		let Feature::HybridFuse { brep, field, op, voxel } = &self.features.get(id.0)?.feature else {
			return None;
		};
		let params = self.effective_params();
		let mut budget = self.features.len().saturating_mul(64).max(1024);
		// Seed the operand stacks with `id` itself so a hand-edited
		// self-referencing fuse is rejected as cyclic instead of recursing.
		let solid = self.build_brep(*brep, &mut vec![id], &mut budget, &params)?;
		let node = self.build(*field, &mut vec![id], &mut budget, &params)?;
		Some(hybrid_boolean(&solid, HybridOperand::Node(&node), *op, voxel.resolve(&params) as f32))
	}
}

/// A bounded **undo/redo snapshot stack** for a [`Document`] — the session-level
/// edit history (distinct from the *feature* history inside the document). A
/// [`Document`] is pure data and cheap to clone, so undo is snapshot-based and
/// therefore covers *every* kind of edit (parameters, features, labels, configs,
/// suppression) with bit-exact restoration: re-evaluating an undone document
/// reproduces the earlier solid exactly (R5 determinism).
///
/// Usage: create with the initial document, [`push`](DocumentHistory::push) a
/// snapshot **after** each completed edit, and navigate with
/// [`undo`](DocumentHistory::undo) / [`redo`](DocumentHistory::redo);
/// [`current`](DocumentHistory::current) is always the live state. Pushing after
/// an undo discards the redo tail (the standard branch-discard semantics), and
/// the stack is bounded: beyond `capacity` snapshots the oldest is dropped.
#[derive(Clone, Debug)]
pub struct DocumentHistory {
	/// The snapshots, oldest first; `snapshots[cursor]` is the current state.
	snapshots: Vec<Document>,
	/// Index of the current snapshot.
	cursor: usize,
	/// Maximum number of snapshots kept (≥ 1).
	capacity: usize,
}

impl DocumentHistory {
	/// A history seeded with `initial` as its only snapshot. `capacity` bounds the
	/// stack (clamped to at least 1 — the current state is always kept).
	pub fn new(initial: Document, capacity: usize) -> Self {
		Self { snapshots: vec![initial], cursor: 0, capacity: capacity.max(1) }
	}

	/// The current document state.
	pub fn current(&self) -> &Document {
		&self.snapshots[self.cursor]
	}

	/// Record `doc` as the new current state: any redo tail (snapshots after the
	/// cursor) is discarded, and if the stack exceeds its capacity the oldest
	/// snapshot is dropped (that state becomes unreachable by undo).
	pub fn push(&mut self, doc: Document) {
		self.snapshots.truncate(self.cursor + 1);
		self.snapshots.push(doc);
		if self.snapshots.len() > self.capacity {
			self.snapshots.remove(0);
		}
		self.cursor = self.snapshots.len() - 1;
	}

	/// Whether an [`undo`](DocumentHistory::undo) can go anywhere.
	pub fn can_undo(&self) -> bool {
		self.cursor > 0
	}

	/// Whether a [`redo`](DocumentHistory::redo) can go anywhere.
	pub fn can_redo(&self) -> bool {
		self.cursor + 1 < self.snapshots.len()
	}

	/// Step back one snapshot and return the (now current) earlier state; `None`
	/// (and no change) at the bottom of the stack.
	pub fn undo(&mut self) -> Option<&Document> {
		if !self.can_undo() {
			return None;
		}
		self.cursor -= 1;
		Some(self.current())
	}

	/// Step forward one snapshot (re-applying an undone edit) and return the now
	/// current state; `None` (and no change) when there is nothing to redo.
	pub fn redo(&mut self) -> Option<&Document> {
		if !self.can_redo() {
			return None;
		}
		self.cursor += 1;
		Some(self.current())
	}
}

/// The single upstream feature a **modifier** operates on, if any — fillet, chamfer,
/// shell, transform, the patterns/mirror, and the wizard cuts (hole, rim fillet,
/// grooves, insert boss). Returns `None` for **generative**
/// features (primitives, booleans, smooth booleans, the hybrid fuse, sketches,
/// lofts/sweeps, catalog parts, lattices/pipes) that have no
/// single input. Used to implement [`Document::set_suppressed`]: a suppressed modifier
/// evaluates to this input.
fn primary_input(f: &Feature) -> Option<FeatureId> {
	match f {
		Feature::Fillet { input, .. }
		| Feature::Chamfer { input, .. }
		| Feature::Shell { input, .. }
		| Feature::Transform { input, .. }
		| Feature::LinearPattern { input, .. }
		| Feature::Mirror { input, .. }
		| Feature::CircularPattern { input, .. }
		| Feature::Hole { input, .. }
		| Feature::CircularRimFillet { input, .. }
		| Feature::ORingGroove { input, .. }
		| Feature::CirclipGroove { input, .. }
		| Feature::HeatsetBoss { input, .. } => Some(*input),
		_ => None,
	}
}

/// Quarter-arc faceting of a [`Feature::CircularRimFillet`] torus band (matches
/// the 8 used by [`kernel_brep::filleted_cylinder`] / [`Feature::FilletedCylinder`]).
const RIM_FILLET_ARC_SEGMENTS: usize = 8;

/// Memory rail of [`Feature::BeamLatticeFill`]: the maximum number of unit
/// cells one fill may instantiate; beyond it the feature fails to evaluate
/// (loud `None`). At the octet's ~36 struts/cell this still allows
/// multi-million-strut graphs — the rail exists so a hand-edited `cell_size`
/// typo (e.g. 0.001 over a 50 mm region) cannot exhaust process memory.
pub const LATTICE_FILL_MAX_CELLS: usize = 100_000;

/// Resolve a `[[Dim; 3]; 2]` corner pair into `(center, half_extent)`. The
/// corners may come in any order — an inverted region is normalized through the
/// `abs`, mirroring the `size.abs()` guard of [`Feature::Box`].
fn resolve_region(params: &HashMap<String, f64>, region: &[[Dim; 3]; 2]) -> (Vec3, Vec3) {
	let a = resolve_vec3(params, &region[0]);
	let b = resolve_vec3(params, &region[1]);
	((a + b) * 0.5, ((b - a) * 0.5).abs())
}

/// Material extent of `solid` measured from `at` along `axis` (the largest AABB-corner
/// projection) — how far a "through everything" [`Feature::Hole`] must bore. `None`
/// for a degenerate axis or when no material lies ahead of `at` (the cut would miss).
fn through_length(solid: &kernel_brep::Solid, at: DVec3, axis: DVec3) -> Option<f64> {
	let axis = axis.try_normalize()?;
	let (lo, hi) = solid.aabb();
	let mut t_max = f64::NEG_INFINITY;
	for i in 0..8 {
		let corner =
			DVec3::new(if i & 1 == 0 { lo.x } else { hi.x }, if i & 2 == 0 { lo.y } else { hi.y }, if i & 4 == 0 { lo.z } else { hi.z });
		t_max = t_max.max((corner - at).dot(axis));
	}
	(t_max > 0.0).then_some(t_max)
}

/// Rewrite every [`FeatureId`] reference inside `feature` through `map` — the id
/// remapping behind [`Document::insert_feature_at`]. Every variant that references
/// earlier features appears here; variants without references are untouched.
fn remap_feature_refs(feature: &mut Feature, map: impl Fn(FeatureId) -> FeatureId) {
	match feature {
		Feature::Boolean { a, b, .. }
		| Feature::SmoothUnion { a, b, .. }
		| Feature::SmoothDifference { a, b, .. }
		| Feature::SmoothIntersection { a, b, .. } => {
			*a = map(*a);
			*b = map(*b);
		}
		Feature::HybridFuse { brep, field, .. } => {
			*brep = map(*brep);
			*field = map(*field);
		}
		Feature::Transform { input, .. }
		| Feature::Fillet { input, .. }
		| Feature::Chamfer { input, .. }
		| Feature::LinearPattern { input, .. }
		| Feature::Mirror { input, .. }
		| Feature::CircularPattern { input, .. }
		| Feature::Shell { input, .. }
		| Feature::Hole { input, .. }
		| Feature::CircularRimFillet { input, .. }
		| Feature::ORingGroove { input, .. }
		| Feature::CirclipGroove { input, .. }
		| Feature::HeatsetBoss { input, .. } => *input = map(*input),
		Feature::Box { .. }
		| Feature::Sphere { .. }
		| Feature::Cylinder { .. }
		| Feature::FilletedCylinder { .. }
		| Feature::ChamferedCylinder { .. }
		| Feature::Gyroid { .. }
		| Feature::GyroidLattice { .. }
		| Feature::Tpms { .. }
		| Feature::BeamLatticeFill { .. }
		| Feature::PipeFeat { .. }
		| Feature::ExtrudeSketch { .. }
		| Feature::LoftSolid { .. }
		| Feature::SweepSolid { .. }
		| Feature::Revolve { .. }
		| Feature::CatalogPart { .. } => {}
	}
}

/// Resolve three [`Dim`]s into a `Vec3` (the implicit side is `f32`).
fn resolve_vec3(params: &HashMap<String, f64>, dims: &[Dim; 3]) -> Vec3 {
	Vec3::new(dims[0].resolve(params) as f32, dims[1].resolve(params) as f32, dims[2].resolve(params) as f32)
}

/// Whether two solids' axis-aligned bounds are strictly disjoint (provably non-overlapping).
/// Used to decide that pattern copies can be combined by exact topology merge
/// ([`kernel_brep::Solid::disjoint_union`]) rather than a boolean union.
fn aabb_disjoint(a: &kernel_brep::Solid, b: &kernel_brep::Solid) -> bool {
	let (amin, amax) = a.aabb();
	let (bmin, bmax) = b.aabb();
	amax.x < bmin.x || bmax.x < amin.x || amax.y < bmin.y || bmax.y < amin.y || amax.z < bmin.z || bmax.z < amin.z
}

/// Build the reflection [`Affine3A`] across the plane through `plane_point` with
/// the given `plane_normal` (need not be unit): `x ↦ x − 2((x−p)·n)n`. Returns the
/// identity for a degenerate normal. Used by the implicit [`Feature::Mirror`] path.
fn reflection_affine(plane_point: Vec3, plane_normal: Vec3) -> Affine3A {
	let n = plane_normal.normalize_or_zero();
	if n.length_squared() < 0.5 {
		return Affine3A::IDENTITY;
	}
	let col = |e: Vec3, nj: f32| e - n * (2.0 * nj);
	let m3 = Mat3::from_cols(col(Vec3::X, n.x), col(Vec3::Y, n.y), col(Vec3::Z, n.z));
	Affine3A::from_mat3_translation(m3, n * (2.0 * plane_point.dot(n)))
}
