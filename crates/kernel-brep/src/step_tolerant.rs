// Copyright (c) LMCAD. Licensed under the MIT License.

//! **Tolerant** STEP import — the read path for real vendor files, where one
//! unreadable face must not cost the other 167 solids.
//!
//! [`import_step_tolerant`] runs the same reconstruction as
//! [`crate::step_import::import_step`] (every route, every fix) but with three
//! differences, all of them *reported* rather than silent:
//!
//! 1. **Per-face failure containment.** A face the exact routes refuse is rolled
//!    back and retried as a **flat repair** (its loops ear-clipped on their own
//!    Newell plane — the boundary chords stay verbatim, so the shell stays
//!    welded; only the face's interior is approximated). A face that cannot even
//!    be repaired is **skipped**, which skips its whole solid (an open shell can
//!    never bind). Both outcomes land in the receipt with the entity id and the
//!    verbatim reason.
//! 2. **Per-solid census with placements.** Every `MANIFOLD_SOLID_BREP` /
//!    `BREP_WITH_VOIDS` in the file is listed as ONE record per assembly
//!    **instance** (the NAUO tree is walked with its `ITEM_DEFINED_TRANSFORMATION`
//!    placements, exactly as OpenCascade's XCAF reader counts solids), named by
//!    its `PRODUCT`, with a world-space bounding box computed **from the entity
//!    geometry** — vertices, exact conic-arc extremes, B-spline control points
//!    (a convex-hull bound, as OpenCascade's `BRepBndLib` uses poles) — so a
//!    solid whose B-rep could not be built still reports its envelope. An
//!    imported solid's box also folds in its reconstructed vertices.
//! 3. **A looser trim-vertex snap.** The B-spline projection allowance is
//!    [`crate::step_import::TOLERANT_SNAP_FACTOR`] × the file's uncertainty
//!    (strict mode: exactly the uncertainty); every vertex accepted beyond the
//!    strict projection tolerance is a `repaired` event.
//!
//! The bound body is the **compound** of every imported instance (each placed
//! by its accumulated assembly placement; a mirrored placement rebuilds the
//! instance with reversed loops so it stays outward-wound), a valid multi-shell
//! [`Solid`]. Strict [`crate::step_import::import_step`] is untouched: it still
//! refuses on the first unreadable face and still imports every brep in its
//! LOCAL frame, ignoring the assembly placements.

use std::collections::{HashMap, HashSet};

use kernel_core::math::{DAffine3, DVec3};

use crate::geom::Curve;
use crate::nurbs::FreeformFace;
use crate::step_import::{
	add_face, add_face_flat, complex_part, edge_sweep, last_enum, parse_with, AssemblyGraph, FaceAccum, Importer, ShellFaces, StepError,
	Value, TOLERANT_SNAP_FACTOR,
};
use crate::topo::{FaceLoops, Solid, VertexId};
use crate::validate::validate;

/// Whether a listed solid made it into the bound body.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SolidStatus {
	/// Reconstructed (possibly with repaired faces), validated, and part of the
	/// compound body.
	Imported,
	/// Listed with its name and envelope only; `reason` says why.
	Skipped,
}

impl SolidStatus {
	pub fn as_str(self) -> &'static str {
		match self {
			SolidStatus::Imported => "imported",
			SolidStatus::Skipped => "skipped",
		}
	}
}

/// One solid instance of the file — a `MANIFOLD_SOLID_BREP` placed by one
/// assembly path (a brep instanced N times yields N records).
#[derive(Clone, Debug)]
pub struct SolidRecord {
	/// The owning `PRODUCT`'s name (`solid #<id>` for a brep outside any product).
	pub name: String,
	/// Product names from the assembly root down to this instance, `/`-joined
	/// (`orphan` for a brep reached by no product).
	pub path: String,
	/// The `MANIFOLD_SOLID_BREP` / `BREP_WITH_VOIDS` entity id.
	pub entity: u32,
	pub status: SolidStatus,
	/// World-space axis-aligned envelope (placement applied).
	pub bbox_min: DVec3,
	pub bbox_max: DVec3,
	/// `"brep"` when the reconstructed solid's vertices are folded in, `"edges"`
	/// when only the entity geometry (vertices, arc extremes, control points)
	/// bounds it — a skipped solid, or one whose faces are all flat.
	pub bbox_source: &'static str,
	/// Faces in the brep's shells, and how many of them were repaired / skipped.
	pub faces: usize,
	pub faces_repaired: usize,
	pub faces_skipped: usize,
	/// Why the solid was skipped (verbatim first failure), `None` when imported.
	pub reason: Option<String>,
}

/// One face/statement-level event of the import: a repair applied or a skip.
#[derive(Clone, Debug)]
pub struct ImportEvent {
	/// The entity id (`0` for an unparseable statement, which has none).
	pub entity: u32,
	/// The entity type (`ADVANCED_FACE`, `MANIFOLD_SOLID_BREP`, `statement`, …).
	pub kind: String,
	/// The product name of the solid the entity belongs to (empty for a statement).
	pub solid: String,
	/// What happened, verbatim.
	pub reason: String,
}

/// The result of [`import_step_tolerant`].
#[derive(Debug)]
pub struct TolerantImport {
	/// The compound of every imported instance, `None` when nothing imported.
	pub solid: Option<Solid>,
	/// The NURBS sidecar of the compound (each patch placed like its instance).
	pub freeform: Vec<FreeformFace>,
	/// Every solid instance of the file, imported or not, in walk order.
	pub solids: Vec<SolidRecord>,
	/// Faces/solids that could not be read, with reasons.
	pub skipped: Vec<ImportEvent>,
	/// Repairs applied: flat-repaired faces, projected trim vertices, skipped
	/// unparseable statements.
	pub repaired: Vec<ImportEvent>,
	/// The file's asserted uncertainty (mm), when it states one.
	pub uncertainty: Option<f64>,
}

impl TolerantImport {
	pub fn imported_count(&self) -> usize {
		self.solids.iter().filter(|s| s.status == SolidStatus::Imported).count()
	}
}

/// One placed occurrence of a brep in the assembly tree.
struct Instance {
	name: String,
	path: String,
	brep: u32,
	placement: DAffine3,
}

/// The reconstruction of one brep entity (shared by all of its instances).
struct BrepBuild {
	solid: Option<Solid>,
	freeform: Vec<FreeformFace>,
	faces: usize,
	repaired: Vec<(u32, String)>,
	skipped: Vec<(u32, String)>,
	reason: Option<String>,
	/// Local-frame geometry bounding primitives (see [`Extent`]).
	extent: Extent,
}

/// Geometry that bounds a brep in its local frame, kept in a form that can be
/// placed exactly: points, plus conic arcs whose axis-extremes are taken AFTER
/// the placement (a rotated arc's extreme is not the image of its local one).
#[derive(Default)]
struct Extent {
	points: Vec<DVec3>,
	arcs: Vec<Arc>,
}

/// A conic arc `c + a·cos t + b·sin t`, `t ∈ [t0, t0 + sweep]` (`a`/`b` carry the
/// semi-axis lengths; an ellipse's are unequal).
#[derive(Clone, Copy)]
struct Arc {
	center: DVec3,
	a: DVec3,
	b: DVec3,
	t0: f64,
	sweep: f64,
}

impl Arc {
	/// The arc placed by `m`.
	fn placed(&self, m: DAffine3) -> Arc {
		Arc {
			center: m.transform_point3(self.center),
			a: m.transform_vector3(self.a),
			b: m.transform_vector3(self.b),
			t0: self.t0,
			sweep: self.sweep,
		}
	}

	/// Fold the arc's axis-aligned extremes into `(min, max)`: for each world axis
	/// the parameter where that coordinate is stationary (and its antipode), when
	/// it lies within the sweep.
	fn bound(&self, min: &mut DVec3, max: &mut DVec3) {
		use std::f64::consts::{PI, TAU};
		let at = |t: f64| self.center + self.a * t.cos() + self.b * t.sin();
		let inside = |t: f64| {
			// Signed offset from t0 along the sweep direction, wrapped to [0, 2π).
			let d = ((t - self.t0) * self.sweep.signum()).rem_euclid(TAU);
			d <= self.sweep.abs() + 1e-9
		};
		for k in 0..3 {
			let (ak, bk) = (self.a[k], self.b[k]);
			if ak.abs() < 1e-300 && bk.abs() < 1e-300 {
				continue;
			}
			let t_star = bk.atan2(ak);
			for t in [t_star, t_star + PI] {
				if inside(t) {
					let p = at(t);
					*min = min.min(p);
					*max = max.max(p);
				}
			}
		}
	}
}

/// Import a STEP file tolerantly — see the module doc for the contract.
///
/// Returns `Err` only when the file has no usable entity graph at all (nothing
/// parses, or no `MANIFOLD_SOLID_BREP` / `ADVANCED_FACE` is present); every
/// other failure is contained and reported in the receipt.
pub fn import_step_tolerant(text: &str) -> Result<TolerantImport, StepError> {
	import_step_tolerant_with(text, true)
}

/// The **census only**: every solid instance of the file with its product name,
/// path and placed envelope from the ENTITY geometry, without reconstructing
/// any B-rep (every record is `Skipped` with reason `census only`, `solid` is
/// `None`). Seconds on a file the full tolerant import takes minutes on — the
/// envelope pass a campaign runs first.
pub fn step_census(text: &str) -> Result<TolerantImport, StepError> {
	import_step_tolerant_with(text, false)
}

fn import_step_tolerant_with(text: &str, reconstruct: bool) -> Result<TolerantImport, StepError> {
	let (ents, issues) = parse_with(text, true)?;
	let mut imp = Importer::new(&ents);
	imp.snap_factor = TOLERANT_SNAP_FACTOR;
	let uncertainty = (imp.uncertainty > 0.0).then_some(imp.uncertainty);

	let mut repaired: Vec<ImportEvent> = issues
		.into_iter()
		.map(|(head, reason)| ImportEvent {
			entity: head.trim_start_matches('#').split(|c: char| !c.is_ascii_digit()).next().and_then(|s| s.parse().ok()).unwrap_or(0),
			kind: "statement".into(),
			solid: String::new(),
			reason: format!("unparseable statement `{head}…` skipped: {reason}"),
		})
		.collect();
	let mut skipped: Vec<ImportEvent> = Vec::new();

	// Every brep entity in the file, ascending — the census baseline.
	let mut all_breps: Vec<u32> =
		ents.iter().filter(|(_, e)| e.name == "MANIFOLD_SOLID_BREP" || e.name == "BREP_WITH_VOIDS").map(|(&id, _)| id).collect();
	all_breps.sort_unstable();

	// Instances from the product tree; orphans (breps no product reaches) at identity.
	let mut instances: Vec<Instance> = Vec::new();
	match AssemblyGraph::resolve(&imp) {
		Ok(graph) => {
			if let Err(e) = enumerate_instances(&graph, &mut instances) {
				repaired.push(ImportEvent {
					entity: 0,
					kind: "assembly".into(),
					solid: String::new(),
					reason: format!("assembly tree walk stopped: {e}; remaining solids listed without placement"),
				});
			}
		}
		Err(e) => repaired.push(ImportEvent {
			entity: 0,
			kind: "assembly".into(),
			solid: String::new(),
			reason: format!("assembly structure unreadable ({e}); solids listed without placement"),
		}),
	}
	let reached: HashSet<u32> = instances.iter().map(|i| i.brep).collect();
	for &brep in &all_breps {
		if !reached.contains(&brep) {
			instances.push(Instance { name: format!("solid #{brep}"), path: "orphan".into(), brep, placement: DAffine3::IDENTITY });
		}
	}
	if instances.is_empty() {
		// A bare-face fragment: one anonymous solid from every ADVANCED_FACE.
		if imp.all_face_ids().is_empty() {
			return Err(StepError::Topology("no MANIFOLD_SOLID_BREP or ADVANCED_FACE entities found".into()));
		}
		instances.push(Instance { name: "solid".into(), path: "fragment".into(), brep: 0, placement: DAffine3::IDENTITY });
	}

	// Reconstruct each brep ONCE (instances share), then place per instance.
	let mut builds: HashMap<u32, BrepBuild> = HashMap::new();
	let mut solids: Vec<SolidRecord> = Vec::with_capacity(instances.len());
	let mut parts: Vec<Solid> = Vec::new();
	let mut freeform: Vec<FreeformFace> = Vec::new();
	for inst in &instances {
		if let std::collections::hash_map::Entry::Vacant(slot) = builds.entry(inst.brep) {
			let build = if reconstruct { build_brep(&imp, inst.brep) } else { census_brep(&imp, inst.brep) };
			for (fid, reason) in &build.repaired {
				repaired.push(ImportEvent { entity: *fid, kind: "ADVANCED_FACE".into(), solid: inst.name.clone(), reason: reason.clone() });
			}
			for (fid, reason) in &build.skipped {
				skipped.push(ImportEvent { entity: *fid, kind: "ADVANCED_FACE".into(), solid: inst.name.clone(), reason: reason.clone() });
			}
			if let (None, Some(reason)) = (&build.solid, &build.reason) {
				skipped.push(ImportEvent {
					entity: inst.brep,
					kind: ents.get(&inst.brep).map(|e| e.name.clone()).unwrap_or_else(|| "MANIFOLD_SOLID_BREP".into()),
					solid: inst.name.clone(),
					reason: reason.clone(),
				});
			}
			slot.insert(build);
		}
		let build = &builds[&inst.brep];
		let mut min = DVec3::splat(f64::INFINITY);
		let mut max = DVec3::splat(f64::NEG_INFINITY);
		for &p in &build.extent.points {
			let q = inst.placement.transform_point3(p);
			min = min.min(q);
			max = max.max(q);
		}
		for arc in &build.extent.arcs {
			arc.placed(inst.placement).bound(&mut min, &mut max);
		}
		let mut bbox_source = "edges";
		let mut status = SolidStatus::Skipped;
		if let Some(local) = &build.solid {
			let placed = place_solid(local, inst.placement);
			let (smin, smax) = placed.aabb();
			min = min.min(smin);
			max = max.max(smax);
			bbox_source = "brep";
			status = SolidStatus::Imported;
			for ff in &build.freeform {
				freeform.push(place_freeform(ff, inst.placement));
			}
			parts.push(placed);
		}
		if !min.x.is_finite() {
			// No geometry at all (an empty brep): an inverted box, as `Solid::aabb`.
			min = DVec3::splat(f64::INFINITY);
			max = DVec3::splat(f64::NEG_INFINITY);
		}
		solids.push(SolidRecord {
			name: inst.name.clone(),
			path: inst.path.clone(),
			entity: inst.brep,
			status,
			bbox_min: min,
			bbox_max: max,
			bbox_source,
			faces: build.faces,
			faces_repaired: build.repaired.iter().filter(|(_, r)| r.contains("flat facets")).count(),
			faces_skipped: build.skipped.len(),
			reason: build.reason.clone(),
		});
	}
	let solid = compound(&parts);
	Ok(TolerantImport { solid, freeform, solids, skipped, repaired, uncertainty })
}

/// Walk the product tree into placed brep instances (mirrors
/// [`crate::step_import::import_step_assembly`]'s traversal, per brep).
fn enumerate_instances(graph: &AssemblyGraph, out: &mut Vec<Instance>) -> Result<(), StepError> {
	const MAX_DEPTH: usize = 64;
	fn emit_rep(
		graph: &AssemblyGraph,
		name: &str,
		path: &str,
		rep: u32,
		at: DAffine3,
		depth: usize,
		out: &mut Vec<Instance>,
	) -> Result<(), StepError> {
		if depth > MAX_DEPTH {
			return Err(StepError::Topology("assembly mapping nests deeper than 64 — the MAPPED_ITEM graph has a cycle".into()));
		}
		let mut breps: Vec<u32> = graph
			.rep_items(rep)?
			.into_iter()
			.filter(|&id| graph.imp.get(id).map(|e| e.name == "MANIFOLD_SOLID_BREP" || e.name == "BREP_WITH_VOIDS").unwrap_or(false))
			.collect();
		breps.sort_unstable();
		for brep in breps {
			out.push(Instance { name: name.to_string(), path: path.to_string(), brep, placement: at });
		}
		for (src, t) in graph.mapped_items(rep)? {
			let src_name = graph
				.imp
				.get(src)?
				.args
				.iter()
				.find_map(|v| v.as_str().filter(|s| !s.is_empty()).map(String::from))
				.unwrap_or_else(|| name.to_string());
			emit_rep(graph, &src_name, path, src, at * t, depth + 1, out)?;
		}
		Ok(())
	}
	fn walk(graph: &AssemblyGraph, pd: u32, at: DAffine3, depth: usize, path: &str, out: &mut Vec<Instance>) -> Result<(), StepError> {
		if depth > MAX_DEPTH {
			return Err(StepError::Topology("assembly tree nests deeper than 64 — the NAUO graph has a cycle".into()));
		}
		let name = graph.product_name(pd).unwrap_or_else(|_| format!("product #{pd}"));
		let here = if path.is_empty() { name.clone() } else { format!("{path}/{name}") };
		if let Some(&rep) = graph.shape_rep.get(&pd) {
			emit_rep(graph, &name, &here, rep, at, depth, out)?;
		}
		let children: Vec<(u32, u32)> =
			graph.nauo.iter().filter(|(_, (parent, _))| *parent == pd).map(|&(id, (_, child))| (id, child)).collect();
		for (nauo_id, child) in children {
			let t = graph.nauo_transform.get(&nauo_id).copied().unwrap_or(DAffine3::IDENTITY);
			walk(graph, child, at * t, depth + 1, &here, out)?;
		}
		Ok(())
	}
	if graph.nauo.is_empty() {
		let mut pds: Vec<u32> = graph.shape_rep.keys().copied().collect();
		pds.sort_unstable();
		for pd in pds {
			walk(graph, pd, DAffine3::IDENTITY, 0, "", out)?;
		}
		return Ok(());
	}
	let children: HashSet<u32> = graph.nauo.iter().map(|&(_, (_, c))| c).collect();
	let mut roots: Vec<u32> = graph.nauo.iter().map(|&(_, (p, _))| p).filter(|p| !children.contains(p)).collect();
	roots.sort_unstable();
	roots.dedup();
	if roots.is_empty() {
		return Err(StepError::Topology("the NAUO graph has no root (every product is someone's child — a cycle)".into()));
	}
	for root in roots {
		walk(graph, root, DAffine3::IDENTITY, 0, "", out)?;
	}
	Ok(())
}

/// The `(face id, reversed)` set of one brep's shells (`0` = every bare face).
fn brep_faces(imp: &Importer, brep: u32) -> Result<ShellFaces, StepError> {
	if brep == 0 {
		return Ok(imp.all_face_ids());
	}
	let e = imp.get(brep)?;
	let outer = e.args.iter().find_map(Value::as_ref).ok_or_else(|| StepError::Parse(format!("#{brep} {} has no outer shell", e.name)))?;
	let mut faces = imp.shell_faces(outer, false)?;
	if e.name == "BREP_WITH_VOIDS" {
		let voids =
			e.args.iter().find_map(Value::as_list).ok_or_else(|| StepError::Parse(format!("#{brep} BREP_WITH_VOIDS has no void list")))?;
		for v in voids {
			let vid = v.as_ref().ok_or_else(|| StepError::Parse(format!("#{brep} void shell is not a reference")))?;
			faces.extend(imp.shell_faces(vid, false)?);
		}
	}
	Ok(faces)
}

/// The census-only build of one brep: face count and entity-geometry extent,
/// no reconstruction ([`step_census`]).
fn census_brep(imp: &Importer, brep: u32) -> BrepBuild {
	let mut build = BrepBuild {
		solid: None,
		freeform: Vec::new(),
		faces: 0,
		repaired: Vec::new(),
		skipped: Vec::new(),
		reason: Some("census only (not reconstructed)".into()),
		extent: Extent::default(),
	};
	match brep_faces(imp, brep) {
		Ok(faces) => {
			build.faces = faces.len();
			build.extent = brep_extent(imp, &faces);
		}
		Err(e) => build.reason = Some(format!("shell structure unreadable: {e}")),
	}
	build
}

/// Reconstruct one brep with per-face containment (see the module doc).
fn build_brep(imp: &Importer, brep: u32) -> BrepBuild {
	let mut build = BrepBuild {
		solid: None,
		freeform: Vec::new(),
		faces: 0,
		repaired: Vec::new(),
		skipped: Vec::new(),
		reason: None,
		extent: Extent::default(),
	};
	let faces = match brep_faces(imp, brep) {
		Ok(f) => f,
		Err(e) => {
			build.reason = Some(format!("shell structure unreadable: {e}"));
			return build;
		}
	};
	build.faces = faces.len();
	build.extent = brep_extent(imp, &faces);
	let mut acc = FaceAccum::default();
	for &(fid, flip) in &faces {
		let cp = acc.checkpoint();
		let first = add_face(imp, fid, flip, &mut acc);
		let Err(e) = first else { continue };
		acc.rollback(cp);
		match add_face_flat(imp, fid, flip, &mut acc) {
			Ok(what) => build.repaired.push((fid, format!("{e}; repaired: {what}"))),
			Err(e2) => {
				acc.rollback(cp);
				build.skipped.push((fid, format!("{e}; flat repair refused: {e2}")));
			}
		}
	}
	// Trim-vertex snaps (accepted under the uncertainty allowance) are repairs too.
	let snaps = std::mem::take(&mut acc.repairs);
	build.repaired.extend(snaps);
	if !build.skipped.is_empty() {
		build.reason = Some(format!("{} of {} faces could not be read — first: {}", build.skipped.len(), build.faces, build.skipped[0].1));
		return build;
	}
	build.freeform = std::mem::take(&mut acc.freeform);
	match acc.finish() {
		Ok(solid) => {
			let v = validate(&solid);
			if v.is_valid() {
				build.solid = Some(solid);
			} else {
				build.reason = Some(format!(
					"reconstructed faces do not form a valid solid: closed={} manifold={} genus={} shells={} (faces={}, repaired={})",
					v.closed,
					v.manifold,
					v.genus,
					v.shells,
					build.faces,
					build.repaired.len()
				));
			}
		}
		Err(e) => build.reason = Some(e.to_string()),
	}
	build
}

/// Local-frame bounding primitives of a brep's faces from the ENTITY geometry:
/// every edge's vertices, conic edges as exact arcs, B-spline edge control
/// points, and B-spline surface control points (convex-hull bounds, as
/// OpenCascade's `BRepBndLib` uses poles). Unreadable pieces are simply left
/// out — the envelope is best-effort by construction.
fn brep_extent(imp: &Importer, faces: &ShellFaces) -> Extent {
	let mut ext = Extent::default();
	let mut seen_edges: HashSet<u32> = HashSet::new();
	let mut seen_surfaces: HashSet<u32> = HashSet::new();
	for &(fid, _) in faces {
		let Ok(face) = imp.get(fid) else { continue };
		let Some(bounds) = face.args.iter().find_map(Value::as_list) else { continue };
		if let Some(surface_ref) = face.args.iter().filter_map(Value::as_ref).next_back() {
			if seen_surfaces.insert(surface_ref) {
				if let Ok(s) = imp.get(surface_ref) {
					if s.name == "B_SPLINE_SURFACE_WITH_KNOTS"
						|| (s.name == "_COMPLEX" && complex_part(&s.args, "B_SPLINE_SURFACE_WITH_KNOTS").is_some())
					{
						if let Ok(surf) = imp.bspline_surface(surface_ref) {
							ext.points.extend(surf.control.iter().flatten().copied());
						}
					}
				}
			}
		}
		for b in bounds {
			let Some(bid) = b.as_ref() else { continue };
			let Ok(be) = imp.get(bid) else { continue };
			let Some(loop_ref) = be.args.iter().find_map(Value::as_ref) else { continue };
			let Ok(lp) = imp.get(loop_ref) else { continue };
			let Some(oriented) = lp.args.iter().find_map(Value::as_list) else { continue };
			for oe in oriented {
				let Some(oe_id) = oe.as_ref() else { continue };
				let Ok(oe_ent) = imp.get(oe_id) else { continue };
				let Some(ec_id) = oe_ent.args.iter().find_map(Value::as_ref) else { continue };
				if !seen_edges.insert(ec_id) {
					continue;
				}
				edge_extent(imp, ec_id, &mut ext);
			}
		}
	}
	ext
}

/// Fold one `EDGE_CURVE`'s bounding geometry into `ext`.
fn edge_extent(imp: &Importer, ec_id: u32, ext: &mut Extent) {
	let Ok(ec) = imp.get(ec_id) else { return };
	if ec.name != "EDGE_CURVE" {
		return;
	}
	let refs: Vec<u32> = ec.args.iter().filter_map(Value::as_ref).collect();
	if refs.len() < 2 {
		return;
	}
	let (Ok(start), Ok(end)) = (imp.vertex(refs[0]), imp.vertex(refs[1])) else { return };
	ext.points.push(start);
	ext.points.push(end);
	let Some(&geom_id) = refs.get(2) else { return };
	let mut gid = geom_id;
	let Ok(mut g) = imp.get(gid) else { return };
	if g.name == "SURFACE_CURVE" || g.name == "SEAM_CURVE" {
		let Some(inner) = g.args.iter().find_map(Value::as_ref) else { return };
		gid = inner;
		let Ok(inner_ent) = imp.get(gid) else { return };
		g = inner_ent;
	}
	let same_sense = last_enum(ec).map(|s| s == "T").unwrap_or(true);
	match g.name.as_str() {
		"CIRCLE" | "ELLIPSE" => {
			let Some(placement) = g.args.iter().find_map(Value::as_ref) else { return };
			let Ok((center, _, x, y)) = imp.frame(placement) else { return };
			let semis: Vec<f64> = g.args.iter().filter_map(Value::as_real).collect();
			let (a1, a2) = if g.name == "CIRCLE" {
				let Some(&r) = semis.last() else { return };
				(r, r)
			} else {
				if semis.len() < 2 {
					return;
				}
				(semis[0], semis[1])
			};
			if !(a1 > 0.0 && a2 > 0.0) {
				return;
			}
			let angle = |p: DVec3| ((p - center).dot(y) / a2).atan2((p - center).dot(x) / a1);
			let Ok(sweep) = edge_sweep(angle(start), angle(end), same_sense, ec_id) else { return };
			ext.arcs.push(Arc { center, a: x * a1, b: y * a2, t0: angle(start), sweep });
		}
		name if name == "B_SPLINE_CURVE_WITH_KNOTS"
			|| (name == "_COMPLEX" && complex_part(&g.args, "B_SPLINE_CURVE_WITH_KNOTS").is_some()) =>
		{
			if let Ok(c) = imp.bspline_curve(gid) {
				ext.points.extend(c.control.iter().copied());
			}
		}
		_ => {}
	}
}

/// Place a reconstructed instance: a proper (det > 0) placement transforms in
/// place; a mirrored one (det < 0) would turn the solid inside out, so it is
/// rebuilt with every loop reversed — outward-wound again — and its analytic
/// tags and conic edge curves carried through the transform.
fn place_solid(local: &Solid, m: DAffine3) -> Solid {
	if m.matrix3.determinant() >= 0.0 {
		return local.transformed(m);
	}
	let positions: Vec<DVec3> = (0..local.vertex_count() as u32).map(|i| m.transform_point3(local.position(VertexId(i)))).collect();
	let faces: Vec<FaceLoops> = local
		.faces()
		.map(|f| {
			let face = local.face(f);
			let loops: Vec<Vec<u32>> = std::iter::once(face.outer)
				.chain(face.inner.iter().copied())
				.map(|lp| {
					let mut vs: Vec<u32> = local.loop_half_edges(lp).into_iter().map(|he| local.half_edge(he).origin.0).collect();
					vs.reverse();
					vs
				})
				.collect();
			FaceLoops { loops, surface: face.surface.transformed(m) }
		})
		.collect();
	let mut out = Solid::from_faces_multiloop(positions, faces);
	for e in local.edges() {
		if let Some(c) = local.edge_curve(e) {
			let he = local.edge(e).half_edge;
			let a = local.half_edge(he).origin;
			let b = local.half_edge(local.half_edge(he).next).origin;
			out.set_edge_curve(a, b, c.transformed(m));
		}
	}
	out
}

/// A freeform sidecar entry placed by `m` (control points and trim rings).
fn place_freeform(ff: &FreeformFace, m: DAffine3) -> FreeformFace {
	let mut surface = ff.surface.clone();
	for row in surface.control.iter_mut() {
		for p in row.iter_mut() {
			*p = m.transform_point3(*p);
		}
	}
	FreeformFace { surface, rings: ff.rings.iter().map(|r| r.iter().map(|&p| m.transform_point3(p)).collect()).collect() }
}

/// The n-ary disjoint union: every part becomes its own shell(s) of ONE solid,
/// with no boolean co-refinement (parts that touch or overlap keep their own
/// shells — the compound is topologically valid regardless). Analytic face tags
/// and conic edge curves are carried over. `None` for no parts.
pub(crate) fn compound(parts: &[Solid]) -> Option<Solid> {
	match parts.len() {
		0 => return None,
		1 => return Some(parts[0].clone()),
		_ => {}
	}
	let mut positions: Vec<DVec3> = Vec::new();
	let mut faces: Vec<FaceLoops> = Vec::new();
	let mut curves: Vec<(VertexId, VertexId, Curve)> = Vec::new();
	for s in parts {
		let base = positions.len() as u32;
		positions.extend((0..s.vertex_count() as u32).map(|i| s.position(VertexId(i))));
		for f in s.faces() {
			let face = s.face(f);
			let loops: Vec<Vec<u32>> = std::iter::once(face.outer)
				.chain(face.inner.iter().copied())
				.map(|lp| s.loop_half_edges(lp).into_iter().map(|he| s.half_edge(he).origin.0 + base).collect())
				.collect();
			faces.push(FaceLoops { loops, surface: face.surface });
		}
		for e in s.edges() {
			if let Some(c) = s.edge_curve(e) {
				let he = s.edge(e).half_edge;
				let a = s.half_edge(he).origin;
				let b = s.half_edge(s.half_edge(he).next).origin;
				curves.push((VertexId(a.0 + base), VertexId(b.0 + base), c));
			}
		}
	}
	let mut out = Solid::from_faces_multiloop(positions, faces);
	for (a, b, c) in curves {
		out.set_edge_curve(a, b, c);
	}
	Some(out)
}
