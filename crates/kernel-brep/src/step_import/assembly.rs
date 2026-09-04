// Copyright (c) LMCAD. Licensed under the MIT License.

//! Assembly structure: the product / shape-representation / NAUO / `MAPPED_ITEM`
//! graph, the placements it carries, and the depth-guarded walk that yields one
//! placed [`Solid`] per brep-bearing component.

use std::collections::HashMap;

use kernel_core::math::{DAffine3, DMat3};

use crate::topo::Solid;

use super::edges::complex_part;
use super::face::{add_face, FaceAccum};
use super::importer::Importer;
use super::parse::{parse, Value};
use super::{import_step, StepError};

// --- Assemblies ----------------------------------------------------------------

/// Maximum component-tree depth walked by [`import_step_assembly`] (a cycle in a
/// malformed NAUO/MAPPED_ITEM graph errors loudly instead of recursing forever).
const ASSEMBLY_MAX_DEPTH: usize = 64;

/// Assembly-structure resolver layered over the entity graph: product names,
/// product-definition → shape-representation links, NAUO child relations with their
/// `ITEM_DEFINED_TRANSFORMATION` placements, and `MAPPED_ITEM` instancing.
pub(crate) struct AssemblyGraph<'a> {
	pub(crate) imp: &'a Importer<'a>,
	/// `PRODUCT_DEFINITION` id → `SHAPE_REPRESENTATION`-family id (via
	/// `PRODUCT_DEFINITION_SHAPE` + `SHAPE_DEFINITION_REPRESENTATION`).
	pub(crate) shape_rep: HashMap<u32, u32>,
	/// NAUO id → `(parent PRODUCT_DEFINITION, child PRODUCT_DEFINITION)`.
	pub(crate) nauo: Vec<(u32, (u32, u32))>,
	/// NAUO id → child→parent placement, from the `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`.
	pub(crate) nauo_transform: HashMap<u32, DAffine3>,
	/// Solids already reconstructed, keyed by their representation id (instances share).
	pub(crate) solid_cache: HashMap<u32, Solid>,
}

impl<'a> AssemblyGraph<'a> {
	/// Resolve the product tree of a parsed file: representation links, NAUO
	/// relations (entity-id order) and their placements. Shared by
	/// [`import_step_assembly`] and the tolerant importer's solid census.
	pub(crate) fn resolve(imp: &'a Importer<'a>) -> Result<Self, StepError> {
		let ents = imp.ents;
		// PRODUCT_DEFINITION → representation links (SHAPE_DEFINITION_REPRESENTATION over
		// PRODUCT_DEFINITION_SHAPE), skipping shape aspects that describe NAUOs. The scan
		// runs in ascending-id order so duplicate links resolve deterministically.
		let mut sdr_ids: Vec<u32> = ents.iter().filter(|(_, e)| e.name == "SHAPE_DEFINITION_REPRESENTATION").map(|(&id, _)| id).collect();
		sdr_ids.sort_unstable();
		let mut shape_rep: HashMap<u32, u32> = HashMap::new();
		for id in sdr_ids {
			let e = &ents[&id];
			let refs: Vec<u32> = e.args.iter().filter_map(Value::as_ref).collect();
			if refs.len() < 2 {
				continue;
			}
			let (pds, rep) = (refs[0], refs[1]);
			let Ok(pds_ent) = imp.get(pds) else { continue };
			if pds_ent.name != "PRODUCT_DEFINITION_SHAPE" {
				continue;
			}
			if let Some(target) = pds_ent.args.iter().find_map(Value::as_ref) {
				if imp.get(target).map(|t| t.name == "PRODUCT_DEFINITION").unwrap_or(false) {
					shape_rep.insert(target, rep);
				}
			}
		}

		// NAUO relations, in entity-id order for determinism.
		let mut nauo: Vec<(u32, (u32, u32))> = Vec::new();
		for (&id, e) in ents.iter() {
			if e.name == "NEXT_ASSEMBLY_USAGE_OCCURRENCE" {
				let refs: Vec<u32> = e.args.iter().filter_map(Value::as_ref).collect();
				if refs.len() < 2 {
					return Err(StepError::Parse(format!("#{id} NEXT_ASSEMBLY_USAGE_OCCURRENCE needs parent and child")));
				}
				nauo.push((id, (refs[0], refs[1])));
			}
		}
		nauo.sort_unstable_by_key(|&(id, _)| id);

		let mut graph = AssemblyGraph { imp, shape_rep, nauo, nauo_transform: HashMap::new(), solid_cache: HashMap::new() };
		let index = NauoIndex::build(imp);
		for (id, (_, child)) in graph.nauo.clone() {
			let child_rep = graph.shape_rep.get(&child).copied();
			if let Some(t) = nauo_placement(imp, &index, id, child_rep)? {
				graph.nauo_transform.insert(id, t);
			}
		}
		Ok(graph)
	}

	/// The product name of a `PRODUCT_DEFINITION`: its formation's product's name
	/// (the first string argument of the `PRODUCT`).
	pub(crate) fn product_name(&self, pd: u32) -> Result<String, StepError> {
		let pd_ent = self.imp.get(pd)?;
		let formation = pd_ent
			.args
			.iter()
			.find_map(Value::as_ref)
			.ok_or_else(|| StepError::Parse(format!("#{pd} PRODUCT_DEFINITION has no formation")))?;
		let product = self
			.imp
			.get(formation)?
			.args
			.iter()
			.find_map(Value::as_ref)
			.ok_or_else(|| StepError::Parse(format!("#{formation} PRODUCT_DEFINITION_FORMATION has no product")))?;
		let name = self.imp.get(product)?.args.iter().find_map(|v| match v {
			Value::Str(s) => Some(s.clone()),
			_ => None,
		});
		Ok(name.unwrap_or_else(|| format!("product #{product}")))
	}

	/// Reconstruct (or fetch from cache) the [`Solid`] of one representation: every
	/// `MANIFOLD_SOLID_BREP`/`BREP_WITH_VOIDS` in its item list, rebuilt through the
	/// same face accumulator as [`import_step`]. `None` when the representation
	/// carries no breps (a pure-placement assembly root).
	pub(crate) fn rep_solid(&mut self, rep: u32) -> Result<Option<Solid>, StepError> {
		if let Some(s) = self.solid_cache.get(&rep) {
			return Ok(Some(s.clone()));
		}
		let mut brep_ids: Vec<u32> = self
			.rep_items(rep)?
			.into_iter()
			.filter(|&id| self.imp.get(id).map(|e| e.name == "MANIFOLD_SOLID_BREP" || e.name == "BREP_WITH_VOIDS").unwrap_or(false))
			.collect();
		brep_ids.sort_unstable();
		if brep_ids.is_empty() {
			return Ok(None);
		}
		let mut acc = FaceAccum::default();
		for brep in brep_ids {
			let e = self.imp.get(brep)?;
			let outer =
				e.args.iter().find_map(Value::as_ref).ok_or_else(|| StepError::Parse(format!("#{brep} {} has no outer shell", e.name)))?;
			let mut faces = self.imp.shell_faces(outer, false)?;
			if e.name == "BREP_WITH_VOIDS" {
				let voids = e
					.args
					.iter()
					.find_map(Value::as_list)
					.ok_or_else(|| StepError::Parse(format!("#{brep} BREP_WITH_VOIDS has no void list")))?;
				for v in voids {
					let vid = v.as_ref().ok_or_else(|| StepError::Parse(format!("#{brep} void shell is not a reference")))?;
					faces.extend(self.imp.shell_faces(vid, false)?);
				}
			}
			for (fid, flip) in faces {
				add_face(self.imp, fid, flip, &mut acc)?;
			}
		}
		let solid = acc.finish()?;
		self.solid_cache.insert(rep, solid.clone());
		Ok(Some(solid))
	}

	/// The item-reference list of a representation entity (the second list argument,
	/// after the name).
	pub(crate) fn rep_items(&self, rep: u32) -> Result<Vec<u32>, StepError> {
		let e = self.imp.get(rep)?;
		Ok(e.args
			.iter()
			.find_map(Value::as_list)
			.ok_or_else(|| StepError::Parse(format!("#{rep} {} has no item list", e.name)))?
			.iter()
			.filter_map(Value::as_ref)
			.collect())
	}

	/// The `MAPPED_ITEM`s of a representation: `(source representation, placement)`
	/// pairs, each placing the source's geometry at the mapped target frame relative
	/// to the map's origin frame.
	pub(crate) fn mapped_items(&self, rep: u32) -> Result<Vec<(u32, DAffine3)>, StepError> {
		let mut out = Vec::new();
		for id in self.rep_items(rep)? {
			let e = self.imp.get(id)?;
			if e.name != "MAPPED_ITEM" {
				continue;
			}
			// MAPPED_ITEM('', #REPRESENTATION_MAP, #target_placement)
			let refs: Vec<u32> = e.args.iter().filter_map(Value::as_ref).collect();
			if refs.len() < 2 {
				return Err(StepError::Parse(format!("#{id} MAPPED_ITEM needs a map and a target placement")));
			}
			let map = self.imp.get(refs[0])?;
			if map.name != "REPRESENTATION_MAP" {
				return Err(StepError::Reference(format!("#{} is {}, expected REPRESENTATION_MAP", refs[0], map.name)));
			}
			// REPRESENTATION_MAP(#origin_placement, #source_representation)
			let map_refs: Vec<u32> = map.args.iter().filter_map(Value::as_ref).collect();
			if map_refs.len() < 2 {
				return Err(StepError::Parse(format!("#{} REPRESENTATION_MAP needs an origin and a representation", refs[0])));
			}
			let origin = placement_affine(self.imp, map_refs[0])?;
			let target = placement_affine(self.imp, refs[1])?;
			out.push((map_refs[1], target * origin.inverse()));
		}
		Ok(out)
	}

	/// Emit one component per brep-bearing representation reachable from `rep`
	/// (its own breps, plus nested `MAPPED_ITEM` instances), placed by `at`.
	fn emit_rep(
		&mut self,
		name: &str,
		rep: u32,
		at: DAffine3,
		depth: usize,
		out: &mut Vec<(String, Solid, DAffine3)>,
	) -> Result<(), StepError> {
		if depth > ASSEMBLY_MAX_DEPTH {
			return Err(StepError::Topology(format!(
				"assembly mapping nests deeper than {ASSEMBLY_MAX_DEPTH} — the MAPPED_ITEM graph has a cycle"
			)));
		}
		if let Some(solid) = self.rep_solid(rep)? {
			out.push((name.to_string(), solid, at));
		}
		for (src, t) in self.mapped_items(rep)? {
			let src_name = self
				.imp
				.get(src)?
				.args
				.iter()
				.find_map(|v| match v {
					Value::Str(s) if !s.is_empty() => Some(s.clone()),
					_ => None,
				})
				.unwrap_or_else(|| name.to_string());
			self.emit_rep(&src_name, src, at * t, depth + 1, out)?;
		}
		Ok(())
	}

	/// Flatten the component tree under `pd`, accumulating placements: leaves (no
	/// child NAUOs) emit their representation's solid; assembly nodes recurse into
	/// their children, and any geometry carried by the node itself is emitted too.
	fn walk(&mut self, pd: u32, at: DAffine3, depth: usize, out: &mut Vec<(String, Solid, DAffine3)>) -> Result<(), StepError> {
		if depth > ASSEMBLY_MAX_DEPTH {
			return Err(StepError::Topology(format!("assembly tree nests deeper than {ASSEMBLY_MAX_DEPTH} — the NAUO graph has a cycle")));
		}
		let name = self.product_name(pd)?;
		if let Some(&rep) = self.shape_rep.get(&pd) {
			self.emit_rep(&name, rep, at, depth, out)?;
		}
		let children: Vec<(u32, u32)> =
			self.nauo.iter().filter(|(_, (parent, _))| *parent == pd).map(|&(id, (_, child))| (id, child)).collect();
		for (nauo_id, child) in children {
			let t = self.nauo_transform.get(&nauo_id).copied().unwrap_or(DAffine3::IDENTITY);
			self.walk(child, at * t, depth + 1, out)?;
		}
		Ok(())
	}
}

/// The affine frame of an `AXIS2_PLACEMENT_3D`: columns `(x, y = axis × x, axis)`
/// with the translation at the placement location — the map from the local frame
/// into world coordinates.
pub(crate) fn placement_affine(imp: &Importer, id: u32) -> Result<DAffine3, StepError> {
	let (origin, axis, x, y) = imp.frame(id)?;
	Ok(DAffine3::from_mat3_translation(DMat3::from_cols(x, y, axis), origin))
}

/// The child→parent placement of one NAUO, from its `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`:
/// the CDSR's relationship complex carries an `ITEM_DEFINED_TRANSFORMATION` mapping
/// frame 1 into frame 2 between `REPRESENTATION_RELATIONSHIP` reps `(rep_1, rep_2)`.
/// When `rep_1` is the CHILD's representation the placement is `frame2 ∘ frame1⁻¹`;
/// writers that store the pair reversed get the inverse. A NAUO with no CDSR places
/// its child at the identity.
/// Sorted entity-id lists the NAUO placement lookup scans — built ONCE per file
/// ([`NauoIndex::build`]) instead of once per NAUO, which on a 45 MB vendor
/// assembly (600 k entities, 219 NAUOs) is the difference between a few
/// hundred milliseconds and a full re-scan of the entity map per relation.
pub(crate) struct NauoIndex {
	/// Every `PRODUCT_DEFINITION_SHAPE` id, ascending.
	pds: Vec<u32>,
	/// Every `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` id, ascending.
	cdsr: Vec<u32>,
}

impl NauoIndex {
	pub(crate) fn build(imp: &Importer) -> Self {
		let mut pds = Vec::new();
		let mut cdsr = Vec::new();
		for (&id, e) in imp.ents.iter() {
			match e.name.as_str() {
				"PRODUCT_DEFINITION_SHAPE" => pds.push(id),
				"CONTEXT_DEPENDENT_SHAPE_REPRESENTATION" => cdsr.push(id),
				_ => {}
			}
		}
		// Ascending-id order so a (malformed) file with duplicate records still
		// resolves deterministically across runs.
		pds.sort_unstable();
		cdsr.sort_unstable();
		NauoIndex { pds, cdsr }
	}
}

pub(crate) fn nauo_placement(imp: &Importer, index: &NauoIndex, nauo: u32, child_rep: Option<u32>) -> Result<Option<DAffine3>, StepError> {
	// Find the PRODUCT_DEFINITION_SHAPE that describes this NAUO…
	let pds_of_nauo = index.pds.iter().copied().find(|&id| imp.ents[&id].args.iter().filter_map(Value::as_ref).any(|r| r == nauo));
	let Some(pds) = pds_of_nauo else { return Ok(None) };
	// …then the CONTEXT_DEPENDENT_SHAPE_REPRESENTATION pointing at that shape aspect.
	for &id in &index.cdsr {
		let e = &imp.ents[&id];
		let refs: Vec<u32> = e.args.iter().filter_map(Value::as_ref).collect();
		if refs.len() < 2 || refs[1] != pds {
			continue;
		}
		// refs[0] is the (usually _COMPLEX) representation relationship.
		let rel = imp.get(refs[0])?;
		let (rel_args, idt_ref) = match rel.name.as_str() {
			"_COMPLEX" => {
				let rr = complex_part(&rel.args, "REPRESENTATION_RELATIONSHIP")
					.ok_or_else(|| StepError::Parse(format!("#{} relationship complex has no REPRESENTATION_RELATIONSHIP", refs[0])))?;
				let rrwt = complex_part(&rel.args, "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION")
					.ok_or_else(|| StepError::Parse(format!("#{} relationship complex has no transformation record", refs[0])))?;
				(rr, rrwt.iter().find_map(Value::as_ref))
			}
			"SHAPE_REPRESENTATION_RELATIONSHIP" | "REPRESENTATION_RELATIONSHIP" => (rel.args.as_slice(), None),
			other => return Err(StepError::Unsupported(format!("NAUO #{nauo}: relationship #{} of type {other}", refs[0]))),
		};
		let Some(idt) = idt_ref else {
			return Ok(Some(DAffine3::IDENTITY)); // an untransformed relationship
		};
		let idt_ent = imp.get(idt)?;
		if idt_ent.name != "ITEM_DEFINED_TRANSFORMATION" {
			return Err(StepError::Unsupported(format!(
				"NAUO #{nauo}: transformation #{idt} of type {} (only ITEM_DEFINED_TRANSFORMATION is importable)",
				idt_ent.name
			)));
		}
		let frames: Vec<u32> = idt_ent.args.iter().filter_map(Value::as_ref).collect();
		if frames.len() < 2 {
			return Err(StepError::Parse(format!("#{idt} ITEM_DEFINED_TRANSFORMATION needs two placements")));
		}
		let f1 = placement_affine(imp, frames[0])?;
		let f2 = placement_affine(imp, frames[1])?;
		let reps: Vec<u32> = rel_args.iter().filter_map(Value::as_ref).collect();
		// rep_1 → rep_2 is frame1 → frame2; orient it child → parent.
		let child_first = match (child_rep, reps.first()) {
			(Some(c), Some(&r1)) => r1 == c,
			_ => true,
		};
		return Ok(Some(if child_first { f2 * f1.inverse() } else { f1 * f2.inverse() }));
	}
	Ok(None)
}

/// Import the **assembly structure** of a STEP file: the flattened component
/// instances as `(product name, part solid, placement)` triples.
///
/// Components come from `NEXT_ASSEMBLY_USAGE_OCCURRENCE` relations (the AP214
/// product tree), each instance placed by its `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`'s
/// `ITEM_DEFINED_TRANSFORMATION` — placements accumulate down nested
/// sub-assemblies, and instances of one part share the same reconstructed geometry
/// (the solid is rebuilt per instance from one cached reconstruction). Files that
/// instance geometry with `MAPPED_ITEM`/`REPRESENTATION_MAP` instead are flattened
/// the same way. A file with NO assembly structure degrades gracefully: every
/// brep-bearing product (or, failing that, the whole file) is returned as a single
/// component at the identity placement, so the function is total over valid
/// part/assembly files. Assembly NODES legitimately carry no geometry of their own
/// (their representation holds only a placement) and contribute no component —
/// only an entire tree without a single brep is an error. Geometry reconstruction
/// is exactly [`import_step`]'s — including every loud [`StepError::Unsupported`]
/// in the module support matrix.
///
/// The placement is a [`DAffine3`] mapping the part's local frame into assembly
/// space; `placement.transform_point3(p)` places a local point.
pub fn import_step_assembly(text: &str) -> Result<Vec<(String, Solid, DAffine3)>, StepError> {
	let ents = parse(text)?;
	let imp = Importer::new(&ents);
	let mut graph = AssemblyGraph::resolve(&imp)?;

	let mut out: Vec<(String, Solid, DAffine3)> = Vec::new();
	if graph.nauo.is_empty() {
		// No assembly tree: emit every brep-bearing product directly (mapped items
		// included), falling back to the whole file as one anonymous component.
		let mut pds: Vec<u32> = graph.shape_rep.keys().copied().collect();
		pds.sort_unstable();
		for pd in pds {
			graph.walk(pd, DAffine3::IDENTITY, 0, &mut out)?;
		}
		if out.is_empty() {
			out.push(("solid".to_string(), import_step(text)?, DAffine3::IDENTITY));
		}
		return Ok(out);
	}

	// Roots: products that parent at least one NAUO but are never a child.
	let children: std::collections::HashSet<u32> = graph.nauo.iter().map(|&(_, (_, c))| c).collect();
	let mut roots: Vec<u32> = graph.nauo.iter().map(|&(_, (p, _))| p).filter(|p| !children.contains(p)).collect();
	roots.sort_unstable();
	roots.dedup();
	if roots.is_empty() {
		return Err(StepError::Topology("the NAUO graph has no root (every product is someone's child — a cycle)".into()));
	}
	for root in roots {
		graph.walk(root, DAffine3::IDENTITY, 0, &mut out)?;
	}
	if out.is_empty() {
		return Err(StepError::Topology("the assembly tree reached no brep-bearing component".into()));
	}
	Ok(out)
}
