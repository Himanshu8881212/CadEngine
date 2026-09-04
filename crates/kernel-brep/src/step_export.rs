// Copyright (c) LMCAD. Licensed under the MIT License.

//! Basic ISO-10303-21 (STEP) export of the analytic B-rep — **AP203**
//! (`CONFIG_CONTROL_DESIGN`, [`export_step`]) and **AP242**
//! ([`export_step_ap242`]) flavors, plus freeform (NURBS) faces
//! ([`export_step_freeform`]) and assembly structure ([`export_step_assembly`]).
//!
//! This fills the interchange gap: the kernel previously only emitted mesh
//! formats. [`export_step`] walks the half-edge topology and writes a textual
//! STEP physical file with an auto-incrementing entity-id allocator
//! (`#1`, `#2`, …). The structure produced is, per face:
//!
//! ```text
//! ADVANCED_FACE → FACE_OUTER_BOUND → EDGE_LOOP
//!              → FACE_BOUND (one per inner/hole loop) → EDGE_LOOP
//!                                     ↳ ORIENTED_EDGE → EDGE_CURVE (LINE)
//!              → <surface> (PLANE / CYLINDRICAL_SURFACE / …) → AXIS2_PLACEMENT_3D
//! ```
//!
//! all collected into a `CLOSED_SHELL` → `MANIFOLD_SOLID_BREP` →
//! `ADVANCED_BREP_SHAPE_REPRESENTATION` with the product / context boilerplate of
//! the chosen application protocol.
//!
//! ## What is approximated
//! - A **circular** edge (a cylinder/cone rim, a sketched circle) exports as a true
//!   `CIRCLE` `EDGE_CURVE`, so those boundaries are geometrically exact. Any OTHER
//!   curved edge (e.g. a B-spline or ellipse seam) still exports as a straight `LINE`
//!   between its two loop vertices — sampled wherever the tessellating constructor
//!   placed them — so such non-circular curved boundaries appear polygonised.
//! - [`Surface::Plane`] and [`Surface::Cylinder`] map to `PLANE` and
//!   `CYLINDRICAL_SURFACE` exactly. [`Surface::Sphere`],
//!   [`Surface::Cone`] and [`Surface::Torus`] map to `SPHERICAL_SURFACE`,
//!   `CONICAL_SURFACE` and `TOROIDAL_SURFACE` respectively. A `CONICAL_SURFACE`
//!   needs a finite base radius and reference plane; we derive that base from
//!   the face centroid's distance along the axis, which is an approximation of
//!   the exact placement but yields a geometrically consistent cone.
//! - [`export_step_freeform`] writes each [`FreeformFace`] sidecar patch as ONE
//!   `ADVANCED_FACE` over a true `B_SPLINE_SURFACE_WITH_KNOTS` (the rational
//!   `_COMPLEX` form when any weight ≠ 1) — exact surface geometry — trimmed by its
//!   recorded rings as `LINE` polylines (the verbatim trim chords, so the loop welds
//!   with the neighbouring faceted faces). The solid's own chord facets lying on a
//!   patch are skipped (they are the patch's tessellation, not extra geometry).
//! - No `pcurves`, no precise edge parameterisation, and no validation
//!   properties are emitted. The intent is a structurally valid file
//!   that round-trips topology and the analytic/NURBS surface geometry.
//!
//! ## Size discipline (face coalescing + entity dedup)
//! - Same-surface facet regions are **coalesced** into one properly-bounded
//!   `ADVANCED_FACE` per region: planes, full cylinder wraps AND full
//!   cone-frustum wraps (each full wrap split at a seam into two half-bands so
//!   no exported face is periodic; a cone region touching its apex is not a
//!   two-rim band and falls back to facets). Output is self-verified per solid
//!   ([`coalesce_roundtrip_ok`]): if the merged body does not round-trip through
//!   our own importer within 0.5% volume, that solid exports fully faceted.
//! - **Entity dedup**: within one solid's emission, identical *geometry* records
//!   (`CARTESIAN_POINT`, `DIRECTION`, `AXIS2_PLACEMENT_3D`, `VECTOR`, `LINE`,
//!   `CIRCLE` and the surface entities) are hash-consed — equal parameters +
//!   placement emit once and share the id. Topology records (`VERTEX_POINT`,
//!   `EDGE_CURVE`, loops, bounds, faces) are never shared, and nothing is
//!   shared across two solids of an assembly (each `MANIFOLD_SOLID_BREP`
//!   subgraph stays self-contained for per-product translators).
//! - `distance_accuracy_value` is **honest**: the worst measured chord sag of
//!   any straight edge this file emitted in place of curved geometry (edge
//!   midpoint's distance to the face's analytic surface), floored at the 1e-6
//!   write precision. A solid whose curved boundaries are all true arcs and
//!   rulings keeps 1e-6. Freeform patch trim chords are NOT measured — their
//!   sag is bounded separately by the import refinement's `PATCH_SAG_TOL`
//!   contract.
//!
//! ## AP242 honesty scope
//! [`export_step_ap242`] emits the **AP242 edition-1 envelope** — the
//! `AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF { 1 0 10303 442 1 1 4 }` file
//! schema, the `'managed model based 3d engineering'` application context /
//! protocol definition, a plain `PRODUCT_DEFINITION_FORMATION` and a
//! `PRODUCT_RELATED_PRODUCT_CATEGORY` — around the SAME ISO-10303-42 geometry
//! entities as the AP203 export (the geometric/topological resources are shared
//! integrated resources, identical across these APs). What this export does **not**
//! claim: no PMI / semantic GD&T annotations, no tessellated or composite shape
//! representations, no kinematics, no validation properties, no draughting — the
//! AP242-specific capability modules are simply absent, which a conforming AP242
//! consumer treats as "not provided".
//!
//! ## Assemblies
//! [`export_step_assembly`] writes a one-level product tree: a root product plus
//! one `NEXT_ASSEMBLY_USAGE_OCCURRENCE` per instance, each placed by an
//! `ITEM_DEFINED_TRANSFORMATION` inside a `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`
//! — exactly the structure [`crate::import_step_assembly`] flattens back.
//! Instances sharing a product name share ONE product and ONE brep (the first
//! instance's geometry); placements must be rigid (rotation + translation — a
//! mirrored or scaled affine is a loud [`StepError::Unsupported`]).

use kernel_core::math::{DAffine3, DVec3};

use crate::geom::{perp_basis, Curve, Surface};
use crate::nurbs::{FreeformFace, NurbsSurface};
use crate::step_import::StepError;
use crate::topo::{FaceId, Solid, VertexId};

/// Monotonic entity-id allocator producing `#1`, `#2`, … and accumulating the
/// DATA-section record text.
struct StepWriter {
	next_id: u32,
	body: String,
	/// Hash-consing cache for shareable GEOMETRY records (points, directions,
	/// placements, vectors, lines, circles, surfaces): exact record text →
	/// already-emitted entity id. `Some` only inside one solid's [`emit_brep`]
	/// (armed at entry, disarmed at exit), so nothing is shared across the
	/// solids of an assembly. A BTreeMap keeps every code path deterministic
	/// (the cache is never iterated, but the discipline is cheap); ids still
	/// allocate in first-encounter order, so output stays byte-stable.
	geom_cache: Option<std::collections::BTreeMap<String, u32>>,
	/// Worst chord sag (model units) of any straight edge emitted in place of
	/// curved geometry so far — measured as the edge midpoint's distance to
	/// the owning face's analytic surface. Drives the honest
	/// `distance_accuracy_value` in [`emit_geom_context_at`].
	max_sag: f64,
}

impl StepWriter {
	fn new() -> Self {
		Self { next_id: 1, body: String::new(), geom_cache: None, max_sag: 0.0 }
	}

	/// Reserve the next entity id without emitting a record (used when an id
	/// must be referenced before — or independently of — its definition).
	fn alloc(&mut self) -> u32 {
		let id = self.next_id;
		self.next_id += 1;
		id
	}

	/// Emit `#id = <record>;` for an already-reserved id and return that id.
	fn emit_with(&mut self, id: u32, record: &str) -> u32 {
		self.body.push_str(&format!("#{id} = {record};\n"));
		id
	}

	/// Allocate a fresh id, emit `#id = <record>;`, and return the id.
	fn emit(&mut self, record: &str) -> u32 {
		let id = self.alloc();
		self.emit_with(id, record)
	}

	/// [`Self::emit`] with hash-consing: while a solid's geometry cache is
	/// armed, an identical record is emitted once and its id reused — the
	/// entity-dedup half of the exporter's size discipline (a 16-arc rim
	/// references ONE `CIRCLE`, all facets of one cylinder ONE
	/// `CYLINDRICAL_SURFACE`). With the cache disarmed this is plain `emit`.
	fn emit_shared(&mut self, record: &str) -> u32 {
		if let Some(&id) = self.geom_cache.as_ref().and_then(|c| c.get(record)) {
			return id;
		}
		let id = self.emit(record);
		if let Some(c) = self.geom_cache.as_mut() {
			c.insert(record.to_string(), id);
		}
		id
	}
}

/// Format an `f64` for a STEP real literal. STEP requires a decimal point, so
/// we always render one (e.g. `1.` rather than `1`).
fn real(x: f64) -> String {
	// Guard against non-finite inputs from degenerate geometry; normalise -0.0.
	let x = if x.is_finite() && x != 0.0 { x } else { 0.0 };
	let s = format!("{x:?}");
	// `{:?}` keeps a decimal point for normal-range values but switches to
	// lowercase scientific (e.g. `1e-6`) for small/large ones — which is NOT a
	// conformant ISO 10303-21 REAL. Rebuild those as `<mantissa-with-dot>E<exp>`.
	if s.contains('e') {
		let sci = format!("{x:e}");
		let (mant, exp) = sci.split_once('e').unwrap_or((sci.as_str(), "0"));
		let mant = if mant.contains('.') { mant.to_string() } else { format!("{mant}.") };
		format!("{mant}E{exp}")
	} else {
		s
	}
}

/// A `DIRECTION` record body for a (unit) vector.
fn direction(name: &str, d: DVec3) -> String {
	format!("DIRECTION('{name}',({},{},{}))", real(d.x), real(d.y), real(d.z))
}

/// A `CARTESIAN_POINT` record body for a point.
fn point(name: &str, p: DVec3) -> String {
	format!("CARTESIAN_POINT('{name}',({},{},{}))", real(p.x), real(p.y), real(p.z))
}

/// Emit an `AXIS2_PLACEMENT_3D` (location + axis `z` + ref direction `x`) and
/// return its entity id. A null normal/ref pair is replaced with the world
/// frame so degenerate surfaces still produce a valid placement.
fn emit_placement(w: &mut StepWriter, location: DVec3, axis: DVec3, ref_dir: DVec3) -> u32 {
	let axis = if axis.length_squared() > 1e-20 { axis.normalize() } else { DVec3::Z };
	let ref_dir = if ref_dir.length_squared() > 1e-20 {
		// Re-orthogonalise the reference direction against the axis.
		let r = ref_dir - axis * ref_dir.dot(axis);
		if r.length_squared() > 1e-20 {
			r.normalize()
		} else {
			perp_basis(axis).0
		}
	} else {
		perp_basis(axis).0
	};

	let loc = w.emit_shared(&point("", location));
	let ax = w.emit_shared(&direction("axis", axis));
	let rf = w.emit_shared(&direction("refdir", ref_dir));
	w.emit_shared(&format!("AXIS2_PLACEMENT_3D('',#{loc},#{ax},#{rf})"))
}

/// Emit the geometric surface for `surface` (placed via an `AXIS2_PLACEMENT_3D`)
/// and return its entity id. `centroid` is the face centroid, used both to
/// orient/place fall-back-free surfaces and to derive a base radius for cones.
fn emit_surface(w: &mut StepWriter, surface: &Surface, centroid: DVec3) -> u32 {
	match *surface {
		Surface::Plane { origin, normal } => {
			let (u, _) = perp_basis(normal.normalize_or_zero());
			let placement = emit_placement(w, origin, normal, u);
			w.emit_shared(&format!("PLANE('',#{placement})"))
		}
		Surface::Cylinder { origin, axis, radius } => {
			let (u, _) = perp_basis(axis.normalize_or_zero());
			let placement = emit_placement(w, origin, axis, u);
			w.emit_shared(&format!("CYLINDRICAL_SURFACE('',#{placement},{})", real(radius)))
		}
		Surface::Sphere { center, radius } => {
			let placement = emit_placement(w, center, DVec3::Z, DVec3::X);
			w.emit_shared(&format!("SPHERICAL_SURFACE('',#{placement},{})", real(radius)))
		}
		Surface::Cone { apex, axis, half_angle } => {
			let axis = axis.normalize_or_zero();
			// CONICAL_SURFACE is placed at a reference plane (the location) with a
			// finite `radius` there and a `semi_angle`. The kernel's cone is given
			// by its apex; we choose the reference plane at the projection of the
			// face centroid onto the axis and compute the matching radius
			// r = h * tan(half_angle), with h the axial distance from the apex.
			let h = (centroid - apex).dot(axis);
			let base_radius = (h.abs() * half_angle.tan()).max(0.0);
			let location = apex + axis * h;
			let (u, _) = perp_basis(axis);
			let placement = emit_placement(w, location, axis, u);
			w.emit_shared(&format!("CONICAL_SURFACE('',#{placement},{},{})", real(base_radius), real(half_angle)))
		}
		Surface::Torus { center, axis, major, minor } => {
			let (u, _) = perp_basis(axis.normalize_or_zero());
			let placement = emit_placement(w, center, axis, u);
			w.emit_shared(&format!("TOROIDAL_SURFACE('',#{placement},{},{})", real(major), real(minor)))
		}
	}
}

/// STEP application-protocol flavor of the product / header boilerplate. The
/// geometry entities are identical (shared ISO-10303-42 integrated resources).
#[derive(Clone, Copy, PartialEq, Eq)]
enum StepFlavor {
	/// AP203 `CONFIG_CONTROL_DESIGN` (1994).
	Ap203,
	/// AP242 edition 1 — envelope only; see the module's "AP242 honesty scope".
	Ap242,
}

/// Compress a full knot vector into STEP's `(distinct knots, multiplicities)`
/// pair — the exact inverse of the importer's `expand_knots`.
fn compress_knots(full: &[f64]) -> (Vec<f64>, Vec<i64>) {
	let mut distinct: Vec<f64> = Vec::new();
	let mut mults: Vec<i64> = Vec::new();
	for &k in full {
		match distinct.last() {
			Some(&last) if last == k => *mults.last_mut().expect("parallel lists") += 1,
			_ => {
				distinct.push(k);
				mults.push(1);
			}
		}
	}
	(distinct, mults)
}

/// Emit the surface entity of a [`FreeformFace`] patch: a plain
/// `B_SPLINE_SURFACE_WITH_KNOTS` when every weight is 1, else the rational
/// `_COMPLEX` instance (`B_SPLINE_SURFACE` + `B_SPLINE_SURFACE_WITH_KNOTS` +
/// `RATIONAL_B_SPLINE_SURFACE` records) — the same forms the importer reads.
fn emit_bspline_surface(w: &mut StepWriter, s: &NurbsSurface) -> u32 {
	let rows: Vec<String> = s
		.control
		.iter()
		.map(|row| {
			let cells: Vec<String> = row.iter().map(|&p| format!("#{}", w.emit(&point("", p)))).collect();
			format!("({})", cells.join(","))
		})
		.collect();
	let grid = rows.join(",");
	let fmt_reals = |v: &[f64]| v.iter().map(|&x| real(x)).collect::<Vec<_>>().join(",");
	let fmt_ints = |v: &[i64]| v.iter().map(i64::to_string).collect::<Vec<_>>().join(",");
	let (ku, mu) = compress_knots(&s.knots_u);
	let (kv, mv) = compress_knots(&s.knots_v);
	let knot_lists = format!("({}),({}),({}),({})", fmt_ints(&mu), fmt_ints(&mv), fmt_reals(&ku), fmt_reals(&kv));
	let (du, dv) = (s.degree_u, s.degree_v);
	let rational = s.weights.iter().flatten().any(|&wt| wt != 1.0);
	if rational {
		let wgrid = s.weights.iter().map(|row| format!("({})", fmt_reals(row))).collect::<Vec<_>>().join(",");
		w.emit(&format!(
			"( BOUNDED_SURFACE() B_SPLINE_SURFACE({du},{dv},({grid}),.UNSPECIFIED.,.F.,.F.,.F.) B_SPLINE_SURFACE_WITH_KNOTS({knot_lists},.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(({wgrid})) REPRESENTATION_ITEM('') SURFACE() )"
		))
	} else {
		w.emit(&format!("B_SPLINE_SURFACE_WITH_KNOTS('',{du},{dv},({grid}),.UNSPECIFIED.,.F.,.F.,.F.,{knot_lists},.UNSPECIFIED.)"))
	}
}

/// Emit the `MANIFOLD_SOLID_BREP` of one solid (every face minus those covered by
/// `patches`, plus one true B-spline `ADVANCED_FACE` per patch) and return its id.
/// Patch trim rings share the solid's `VERTEX_POINT`s and `EDGE_CURVE`s wherever
/// their chord positions intern to solid vertices, so the patch faces stay
/// edge-paired with their faceted neighbours.
/// A coalesced same-surface region: several tessellation facets that share one
/// analytic surface, replaced on export by ONE properly-bounded `ADVANCED_FACE`
/// (or two half-band faces for a full cylinder/cone-frustum wrap). This is the
/// export-side face re-coalescing of FRICTION #20: per-facet curved faces
/// bounded by chord quads are technically off-surface and make third-party
/// translators (OCC, Onshape) crawl through millions of heal operations; a
/// merged face has 10-50x fewer entities and its boundary (rim arcs +
/// rulings) lies EXACTLY on the surface. Any region that fails a validity
/// check falls back to the faceted path — never a guess.
struct CoalescedRegion {
	rep: FaceId,
	loops: Vec<Vec<u32>>, // directed vertex rings, facet-inherited winding
}

/// Group the solid's faces into coalescible same-surface regions. Returns the
/// regions plus the set of face ids they cover (skipped by the per-facet path).
/// Planes coalesce whenever their boundary chains into closed loops; cylinders
/// and cone FRUSTA (apex-free — a region touching the cone apex falls back)
/// additionally require every boundary vertex on the tagged surface, and a
/// FULL wrap with two clean rims is split into two half-bands at a seam so no
/// face is periodic.
fn coalesce_regions(
	solid: &Solid,
	vert_pair_curve: &mut std::collections::HashMap<(u32, u32), Curve>,
	skip: &dyn Fn(FaceId) -> bool,
) -> (Vec<CoalescedRegion>, std::collections::HashSet<u32>) {
	use std::collections::{HashMap, HashSet};
	let q = |x: f64| (x * 1e5).round() as i64;
	let canon_dir = |d: DVec3| -> (DVec3, f64) {
		let d = d.normalize_or_zero();
		let flip = if d.x.abs() > 1e-7 {
			d.x < 0.0
		} else if d.y.abs() > 1e-7 {
			d.y < 0.0
		} else {
			d.z < 0.0
		};
		(if flip { -d } else { d }, if flip { -1.0 } else { 1.0 })
	};
	#[derive(PartialEq, Eq, Hash, PartialOrd, Ord)]
	enum Key {
		Plane(i64, i64, i64, i64),
		Cyl(i64, i64, i64, i64, i64, i64, i64),
		Cone(i64, i64, i64, i64, i64, i64, i64),
	}
	let mut groups: HashMap<Key, Vec<FaceId>> = HashMap::new();
	for fid in solid.faces() {
		if solid.face_vertices(fid).len() < 3 || skip(fid) {
			continue;
		}
		let key = match solid.face(fid).surface {
			Surface::Plane { origin, normal } => {
				// offset uses the CANON normal directly: origin·n_canon separates the
				// two opposite faces of a slab (multiplying by the flip sign merged a
				// box's top with its bottom — caught by the round-trip suite)
				let (n, _) = canon_dir(normal);
				Key::Plane(q(n.x), q(n.y), q(n.z), q(n.dot(origin) * 0.1))
			}
			Surface::Cylinder { origin, axis, radius } => {
				let (a, _) = canon_dir(axis);
				let anchor = origin - a * origin.dot(a);
				Key::Cyl(q(a.x), q(a.y), q(a.z), q(anchor.x * 0.1), q(anchor.y * 0.1), q(anchor.z * 0.1), q(radius * 0.1))
			}
			Surface::Cone { apex, axis, half_angle } => {
				// The cone axis is NOT canonicalised: its orientation (apex→body)
				// distinguishes the two nappes, so same-cone faces share the exact
				// builder axis while a mirrored cone stays a different key.
				let a = axis.normalize_or_zero();
				Key::Cone(q(a.x), q(a.y), q(a.z), q(apex.x * 0.1), q(apex.y * 0.1), q(apex.z * 0.1), q(half_angle))
			}
			_ => continue,
		};
		groups.entry(key).or_default().push(fid);
	}

	let dbg = std::env::var_os("LMCAD_COAL_DEBUG").is_some();
	let mut region_covers: Vec<Vec<u32>> = Vec::new();
	// diagnostic / safety valve: emit fully faceted faces (no analytic merging)
	if std::env::var_os("LMCAD_COAL_OFF").is_some() {
		return (Vec::new(), std::collections::HashSet::new());
	}
	let mut regions: Vec<CoalescedRegion> = Vec::new();
	let mut covered: HashSet<u32> = HashSet::new();
	// Same-surface faces may form several DISJOINT patches (two pads on one
	// plane); merging those would misread the second outer loop as a hole. Split
	// each group into edge-connected components and coalesce per component.
	let mut components: Vec<(bool, Vec<FaceId>)> = Vec::new(); // (is_band, faces)
															// deterministic order: HashMap iteration once made the emitted file (and a
															// wrong-outer-loop bug below) vary run to run
	let mut groups_v: Vec<(Key, Vec<FaceId>)> = groups.into_iter().collect();
	groups_v.sort_by(|a, b| a.0.cmp(&b.0));
	for (key, fids) in groups_v {
		if fids.len() < 2 {
			continue;
		}
		let mut edge_owner: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
		for (i, &fid) in fids.iter().enumerate() {
			let face = solid.face(fid);
			for lid in std::iter::once(face.outer).chain(face.inner.iter().copied()) {
				let verts: Vec<u32> = solid.loop_half_edges(lid).into_iter().map(|he| solid.half_edge(he).origin.0).collect();
				let n = verts.len();
				for k in 0..n {
					let (a, b) = (verts[k], verts[(k + 1) % n]);
					if a != b {
						edge_owner.entry(if a < b { (a, b) } else { (b, a) }).or_default().push(i);
					}
				}
			}
		}
		let mut parent: Vec<usize> = (0..fids.len()).collect();
		fn find(p: &mut [usize], mut i: usize) -> usize {
			while p[i] != i {
				p[i] = p[p[i]];
				i = p[i];
			}
			i
		}
		for owners in edge_owner.values() {
			for w2 in owners.windows(2) {
				let (ra, rb) = (find(&mut parent, w2[0]), find(&mut parent, w2[1]));
				parent[ra] = rb;
			}
		}
		let mut comp: HashMap<usize, Vec<FaceId>> = HashMap::new();
		for (i, &fid) in fids.iter().enumerate() {
			comp.entry(find(&mut parent, i)).or_default().push(fid);
		}
		let mut comps: Vec<Vec<FaceId>> = comp.into_values().collect();
		comps.sort_by_key(|cf| cf.iter().map(|f| f.0).min());
		for cf in comps {
			if cf.len() >= 2 {
				components.push((matches!(key, Key::Cyl(..) | Key::Cone(..)), cf));
			}
		}
	}
	'group: for (is_band, fids) in components {
		if dbg {
			let kind = match solid.face(fids[0]).surface {
				Surface::Plane { .. } => "plane",
				Surface::Cylinder { .. } => "cyl",
				Surface::Cone { .. } => "cone",
				_ => "?",
			};
			eprintln!("group {} faces, kind {kind}", fids.len());
		}
		// Directed boundary edges: an undirected pair used once is boundary; used
		// twice (once per direction) is interior; anything else is non-manifold
		// within the group -> fall back.
		let mut undirected: HashMap<(u32, u32), u32> = HashMap::new();
		let mut directed: HashSet<(u32, u32)> = HashSet::new();
		for &fid in &fids {
			let face = solid.face(fid);
			for lid in std::iter::once(face.outer).chain(face.inner.iter().copied()) {
				let verts: Vec<u32> = solid.loop_half_edges(lid).into_iter().map(|he| solid.half_edge(he).origin.0).collect();
				let n = verts.len();
				for k in 0..n {
					let (a, b) = (verts[k], verts[(k + 1) % n]);
					if a == b {
						continue;
					}
					if !directed.insert((a, b)) {
						if dbg {
							eprintln!("  abort: dup directed edge");
						}
						continue 'group; // duplicated directed edge
					}
					*undirected.entry(if a < b { (a, b) } else { (b, a) }).or_insert(0) += 1;
				}
			}
		}
		let boundary: Vec<(u32, u32)> =
			directed.iter().copied().filter(|&(a, b)| undirected[&if a < b { (a, b) } else { (b, a) }] == 1).collect();
		if boundary.len() < 3 {
			continue;
		}
		// Chain into closed loops (each vertex must have exactly one outgoing edge).
		let mut succ: HashMap<u32, u32> = HashMap::new();
		for &(a, b) in &boundary {
			if succ.insert(a, b).is_some() {
				if dbg {
					eprintln!("  abort: branching boundary");
				}
				continue 'group;
			}
		}
		let mut remaining: HashSet<u32> = succ.keys().copied().collect();
		let mut loops: Vec<Vec<u32>> = Vec::new();
		// deterministic chain start (min vertex id): the seam choice downstream
		// depends on ring[0], and an arbitrary HashSet start made the emitted
		// file vary run to run — unsound under the round-trip self-verification
		while let Some(start) = remaining.iter().copied().min() {
			let mut ring = vec![start];
			remaining.remove(&start);
			let mut cur = succ[&start];
			while cur != start {
				if !remaining.remove(&cur) {
					if dbg {
						eprintln!("  abort: open chain");
					}
					continue 'group; // open chain / crossing
				}
				ring.push(cur);
				cur = match succ.get(&cur) {
					Some(&n) => n,
					None => continue 'group,
				};
			}
			if ring.len() < 3 {
				continue 'group;
			}
			loops.push(ring);
		}
		if loops.is_empty() {
			continue;
		}

		if !is_band {
			{
				// The FIRST loop is emitted as FACE_OUTER_BOUND — but chain order
				// came from a HashSet and was ARBITRARY: a hole loop landing first
				// emitted the face inside-out (watertight import, wrong volume,
				// varying run to run). Order by |enclosed area|, outer first, and
				// self-check the net area against the facets before trusting it.
				let normal = match solid.face(fids[0]).surface {
					Surface::Plane { normal, .. } => normal.normalize_or_zero(),
					_ => continue,
				};
				let helper = if normal.x.abs() < 0.9 { DVec3::X } else { DVec3::Y };
				let bu = normal.cross(helper).normalize();
				let bv = normal.cross(bu);
				let ring_area = |ring: &[u32]| -> f64 {
					let mut s = 0.0;
					for k in 0..ring.len() {
						let p = solid.position(VertexId(ring[k]));
						let q2 = solid.position(VertexId(ring[(k + 1) % ring.len()]));
						s += p.dot(bu) * q2.dot(bv) - q2.dot(bu) * p.dot(bv);
					}
					0.5 * s
				};
				loops.sort_by(|a, b| ring_area(b).abs().partial_cmp(&ring_area(a).abs()).unwrap());
				let net_loops: f64 = ring_area(&loops[0]).abs() - loops[1..].iter().map(|l| ring_area(l).abs()).sum::<f64>();
				let net_faces: f64 = fids
					.iter()
					.map(|&fid| {
						let face = solid.face(fid);
						let outer: Vec<u32> =
							solid.loop_half_edges(face.outer).into_iter().map(|he| solid.half_edge(he).origin.0).collect();
						let inner: f64 = face
							.inner
							.iter()
							.map(|&lid| {
								let r: Vec<u32> = solid.loop_half_edges(lid).into_iter().map(|he| solid.half_edge(he).origin.0).collect();
								ring_area(&r).abs()
							})
							.sum();
						ring_area(&outer).abs() - inner
					})
					.sum();
				if (net_loops - net_faces).abs() > 0.01 * net_faces.abs().max(1e-6) {
					if dbg {
						eprintln!("  abort: merged plane area {net_loops:.3} != facet area {net_faces:.3}");
					}
					continue;
				}
				regions.push(CoalescedRegion { rep: fids[0], loops });
				region_covers.push(fids.iter().map(|f| f.0).collect());
				covered.extend(fids.iter().map(|f| f.0));
			}
		} else {
			{
				// Rotational band: CYLINDERS (constant radius) and CONE FRUSTA
				// (radius linear in the axial height above the apex, r(h) = h·tan α)
				// share one discipline — angle-about-axis winding, two full rims, a
				// seam split into two half-bands. `anchor` is a point on the axis
				// (the cylinder origin / the cone apex); `radius_at` the local rim
				// radius law.
				let (anchor, axis, r0, slope) = match solid.face(fids[0]).surface {
					Surface::Cylinder { origin, axis, radius } => (origin, axis.normalize_or_zero(), radius, 0.0),
					Surface::Cone { apex, axis, half_angle } => (apex, axis.normalize_or_zero(), 0.0, half_angle.tan()),
					_ => continue,
				};
				let radius_at = |h: f64| r0 + h * slope;
				let hz = |vid: u32| (solid.position(VertexId(vid)) - anchor).dot(axis);
				let radial_err = |vid: u32| {
					let d = solid.position(VertexId(vid)) - anchor;
					let h = d.dot(axis);
					((d - axis * h).length() - radius_at(h)).abs()
				};
				// Apex-free frusta ONLY: a cone region with a vertex at (or behind)
				// the apex is a tip cap, not a two-rim band — a merged face there
				// would carry the apex singularity on its boundary. Fall back.
				if slope != 0.0 {
					let hs: Vec<f64> =
						fids.iter().flat_map(|&fid| solid.face_vertices(fid)).map(|v| (solid.position(v) - anchor).dot(axis)).collect();
					let hmax = hs.iter().copied().fold(0.0_f64, f64::max);
					if hs.iter().any(|&h| h <= 1e-6 * (1.0 + hmax.abs())) {
						if dbg {
							eprintln!("  abort: cone region touches its apex");
						}
						continue 'group;
					}
				}
				// Two tolerances, both following the LOCAL rim radius (constant on a
				// cylinder, h·tan α on a cone): boundary vertices belong to facets
				// TAGGED with this surface, so they sit within the chordal sag of the
				// faceted band — accept that band as membership; but only synthesize
				// a true rim ARC between vertices exactly on the circle
				// (heal-inserted mid-chord vertices keep chord LINEs — no worse than
				// the faceted export).
				let band_tol = |r: f64| 0.05 * r + 1e-4;
				let circ_tol = |r: f64| 1e-5 * (1.0 + r);
				let on_surf = |vid: u32| radial_err(vid) < band_tol(radius_at(hz(vid)));
				let mut synth: Vec<((u32, u32), Curve)> = Vec::new();
				for &(a, b) in &boundary {
					if !on_surf(a) || !on_surf(b) {
						if dbg {
							eprintln!(
								"  abort: boundary vertex off surface by {:.5} / {:.5} (local r {:.3})",
								radial_err(a),
								radial_err(b),
								radius_at(hz(a))
							);
						}
						continue 'group;
					}
					let pair = if a < b { (a, b) } else { (b, a) };
					let is_arc = matches!(vert_pair_curve.get(&pair), Some(Curve::Circle { .. }));
					if !is_arc {
						let (za, zb) = (hz(a), hz(b));
						let r_rim = radius_at(0.5 * (za + zb));
						if (za - zb).abs() < 1e-6 * (1.0 + r_rim) && radial_err(a) < circ_tol(r_rim) && radial_err(b) < circ_tol(r_rim) {
							// an untagged rim CHORD (boolean-cut rims land on chords —
							// the known SSI-seam frontier — and REVOLVE rims carry no
							// tag at all): both endpoints sit on the tagged surface's
							// rim circle at one height, so the true arc is
							// unambiguous; reconstruct it. The shared EDGE_CURVE also
							// snaps the neighbouring planar face's hole loop to the
							// same circle — a consistent watertight boundary.
							synth.push((pair, Curve::Circle { center: anchor + axis * za, normal: axis, radius: radius_at(za) }));
						}
						// else: chord (heal-subdivided rim) or ruling — both stay LINEs.
					}
				}
				for (pair, c) in synth {
					vert_pair_curve.entry(pair).or_insert(c);
				}
				// Angular winding decides the topology: a loop winding ±2π is a full
				// rim, so the region wraps the surface and MUST be split at a seam
				// (a periodic face, or a curved face whose second rim is an "inner"
				// loop, chokes importers). No full winding -> a plain bounded patch,
				// but only with a single loop (curved faces with holes are not
				// importable); anything else falls back to facets.
				let (u, v) = perp_basis(axis);
				let ang = |vid: u32| {
					let d = solid.position(VertexId(vid)) - anchor;
					d.dot(v).atan2(d.dot(u))
				};
				let wrap = |x: f64| (x + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI;
				let winding = |ring: &[u32]| -> f64 {
					let n = ring.len();
					(0..n).map(|i| wrap(ang(ring[(i + 1) % n]) - ang(ring[i]))).sum()
				};
				let winds: Vec<f64> = loops.iter().map(|r| winding(r)).collect();
				let full: Vec<usize> =
					winds.iter().enumerate().filter(|(_, w)| w.abs() > std::f64::consts::TAU - 0.1).map(|(i, _)| i).collect();
				if full.is_empty() {
					// A bounded curved patch can have an arbitrarily-shaped boundary
					// whose cylinder unwrap is not u-monotone — the importer (and
					// some translators) cannot triangulate that. Conservative rule:
					// only the clean two-rim full wrap merges; everything else keeps
					// the faceted path.
					if dbg {
						eprintln!("  skip: bounded curved region ({} loops) stays faceted", loops.len());
					}
					continue;
				}
				if dbg {
					let ws: Vec<String> = winds.iter().map(|w| format!("{:.2}", w / std::f64::consts::TAU)).collect();
					let kind = if slope == 0.0 { format!("cyl r={r0:.2}") } else { format!("cone tan={slope:.2}") };
					eprintln!("  {kind} loops={} windings(turns)={:?} full={:?}", loops.len(), ws, full);
				}
				if loops.len() != 2 || full.len() != 2 {
					if dbg {
						eprintln!("  abort: full wrap with {} loops / {} full rims", loops.len(), full.len());
					}
					continue;
				}

				let rim_a = &loops[0];
				let rim_b = &loops[1];
				let a0 = 0usize;
				let target180 = wrap(ang(rim_a[0]) + std::f64::consts::PI);
				let near = |ring: &[u32], target: f64| -> Option<usize> {
					let (mut best, mut err) = (0usize, f64::INFINITY);
					for (i, &vid) in ring.iter().enumerate() {
						let e = wrap(ang(vid) - target).abs();
						if e < err {
							err = e;
							best = i;
						}
					}
					if err < 1e-6 {
						Some(best)
					} else {
						None
					}
				};
				let (Some(a180), Some(b0), Some(b180)) = (near(rim_a, target180), near(rim_b, ang(rim_a[0])), near(rim_b, target180))
				else {
					if dbg {
						eprintln!("  abort: no angle-matched seam vertices");
					}
					continue;
				};
				let seg = |ring: &[u32], from: usize, to: usize| -> Vec<u32> {
					let n = ring.len();
					let mut out = vec![ring[from]];
					let mut i = from;
					while i != to {
						i = (i + 1) % n;
						out.push(ring[i]);
					}
					out
				};
				// two half loops: rimA[a0..a180] + seam + rimB[b180..b0] + seam, and complements
				let mk = |a_from: usize, a_to: usize, b_from: usize, b_to: usize| -> Vec<u32> {
					let mut ring = seg(rim_a, a_from, a_to);
					let mut tail = seg(rim_b, b_from, b_to);
					ring.append(&mut tail);
					ring
				};
				let half1 = mk(a0, a180, b180, b0);
				let half2 = mk(a180, a0, b0, b180);
				if half1.len() < 3 || half2.len() < 3 {
					continue;
				}
				regions.push(CoalescedRegion { rep: fids[0], loops: vec![half1] });
				regions.push(CoalescedRegion { rep: fids[0], loops: vec![half2] });
				region_covers.push(fids.iter().map(|f| f.0).collect());
				region_covers.push(Vec::new()); // faces credited to the first half
				covered.extend(fids.iter().map(|f| f.0));
			}
		}
	}
	if let Some(lim) = std::env::var("LMCAD_COAL_LIMIT").ok().and_then(|s| s.parse::<usize>().ok()) {
		// diagnostic: keep only the first `lim` merged regions (drop the rest
		// back to facets) — used to bisect a bad merge
		while regions.len() > lim {
			regions.pop();
			if let Some(fs) = region_covers.pop() {
				for f in fs {
					covered.remove(&f);
				}
			}
		}
	}
	(regions, covered)
}

fn emit_brep(w: &mut StepWriter, solid: &Solid, patches: &[FreeformFace], product_name: &str, coalesce: bool) -> u32 {
	use std::collections::HashMap;
	let key = |p: DVec3| (p.x.to_bits(), p.y.to_bits(), p.z.to_bits());

	// Arm the geometry hash-consing cache: identical geometry records within
	// THIS solid share one entity (see the module's "Size discipline"). The
	// cache is disarmed before returning so nothing is shared across the
	// solids of an assembly.
	w.geom_cache = Some(Default::default());

	// --- Per-vertex CARTESIAN_POINT + VERTEX_POINT ---------------------------
	// `positions` runs parallel to `vertex_point` and outgrows the solid when a
	// patch ring needs a vertex the solid never interned.
	let vertex_count = solid.vertex_count();
	let mut positions: Vec<DVec3> = (0..vertex_count).map(|i| solid.position(VertexId(i as u32))).collect();
	let mut vertex_point = vec![0u32; vertex_count];
	let mut vert_index: HashMap<(u64, u64, u64), u32> = HashMap::new();
	for (i, slot) in vertex_point.iter_mut().enumerate() {
		let p = positions[i];
		let cp = w.emit_shared(&point("", p));
		*slot = w.emit(&format!("VERTEX_POINT('',#{cp})"));
		vert_index.entry(key(p)).or_insert(i as u32);
	}

	// --- Per-edge CIRCLE / LINE EDGE_CURVE, deduplicated by undirected vertex pair ---
	// An edge carrying an analytic circle exports as a true CIRCLE; every other edge is a
	// straight LINE between its loop vertices (see module note on approximation). We cache
	// EDGE_CURVEs keyed by the sorted vertex pair so a shared edge produces one EDGE_CURVE
	// referenced by two ORIENTED_EDGEs.
	let mut edge_curves: HashMap<(u32, u32), u32> = HashMap::new();

	// Analytic curve carried by each edge (e.g. a cylinder/cone rim circle), keyed by the
	// sorted vertex pair — so a circular edge exports as a true CIRCLE arc, not a chord.
	let mut vert_pair_curve: HashMap<(u32, u32), Curve> = HashMap::new();
	for e in solid.edges() {
		if let Some(c) = solid.edge_curve(e) {
			let he = solid.edge(e).half_edge;
			let a = solid.half_edge(he).origin.0;
			let b = solid.half_edge(solid.half_edge(he).next).origin.0;
			vert_pair_curve.insert(if a < b { (a, b) } else { (b, a) }, c);
		}
	}

	// --- Freeform patch coverage ----------------------------------------------
	// A solid face is a patch facet — skipped in favour of the patch's own
	// ADVANCED_FACE — iff its every outer-loop vertex lies ON one patch (Newton
	// projection within the importer's trim-vertex tolerance) AND its centroid lies
	// within the facet SAG tolerance of that patch (a control-net AABB pre-filter
	// keeps the test cheap). Patch facets satisfy this by construction — boundary
	// chords on the patch, interior vertices evaluated on it — but their CENTROIDS
	// chord below the curved surface by up to the refinement's `PATCH_SAG_TOL`, so
	// the centroid test must use that contract (with a 2× constant for the
	// facet-interior vs edge-midpoint gap), NOT the vertex projection tolerance: at
	// 1e-6 every curved facet fails and the file double-covers the patch with its
	// own tessellation. The centroid test still rejects a genuinely distinct face
	// spanning the same trim ring (e.g. a flat cap under a bulge), which sits off
	// the patch by the bulge, not by a facet sag.
	/// Per-patch projection probe: Newton seed grid, control-net AABB, patch scale.
	struct PatchProbe {
		seeds: Vec<(kernel_core::math::DVec2, DVec3)>,
		lo: DVec3,
		hi: DVec3,
		scale: f64,
	}
	let patch_probe: Vec<PatchProbe> = patches
		.iter()
		.map(|p| {
			let mut lo = DVec3::splat(f64::INFINITY);
			let mut hi = DVec3::splat(f64::NEG_INFINITY);
			let mut scale = 0.0_f64;
			for q in p.surface.control.iter().flatten() {
				lo = lo.min(*q);
				hi = hi.max(*q);
				scale = scale.max(q.length());
			}
			PatchProbe { seeds: p.surface.projection_seeds(crate::step_import::PATCH_SEED_GRID), lo, hi, scale: 1.0 + scale }
		})
		.collect();
	// `tol` is relative to the PATCH scale (matching the refinement's sag budget,
	// which is relative to the face's own scale — NOT to the probed point's norm,
	// which can be much smaller near the origin); [`NurbsSurface::project`] scales
	// its tolerance by `1 + |q|`, so convert.
	let on_patch = |pi: usize, q: DVec3, tol: f64| -> bool {
		let probe = &patch_probe[pi];
		let abs = tol * probe.scale;
		let pad = DVec3::splat(abs);
		// The control net's convex hull bounds the patch, so its AABB (inflated by
		// the tolerance) bounds every on-patch point.
		if q.cmplt(probe.lo - pad).any() || q.cmpgt(probe.hi + pad).any() {
			return false;
		}
		patches[pi].surface.project(&probe.seeds, q, abs / (1.0 + q.length())).is_some()
	};
	let covered = |fid: FaceId| -> bool {
		let verts = solid.face_vertices(fid);
		let centroid = face_centroid(solid, fid);
		(0..patches.len()).any(|pi| {
			on_patch(pi, centroid, 2.0 * crate::step_import::PATCH_SAG_TOL)
				&& verts.iter().all(|&v| on_patch(pi, solid.position(v), crate::step_import::PATCH_PROJECT_TOL))
		})
	};

	// --- Analytic face coalescing (FRICTION #20 at export) --------------------
	// Patch-covered facets are excluded: they are replaced by the B-spline
	// patch's own ADVANCED_FACE, and coalescing them too would double-cover.
	let (regions, merged_away) = if coalesce {
		coalesce_regions(solid, &mut vert_pair_curve, &|fid| !patches.is_empty() && covered(fid))
	} else {
		(Vec::new(), std::collections::HashSet::new())
	};

	// --- Honest distance_accuracy_value: measure the real chord sag ----------
	// Any straight LINE edge on a CURVED analytic face whose midpoint sits off
	// that face's surface is a chord standing in for curved geometry (an
	// untagged rim, a boolean seam polyline, a sphere/torus facet edge). Record
	// the worst offset so the file's uncertainty states what was actually
	// written instead of a blanket 1e-6. Rim arcs are exact and rulings lie ON
	// the surface (both contribute ~0), so an all-analytic-boundary solid keeps
	// the 1e-6 floor. Freeform patch trim chords are NOT measured (their sag is
	// bounded separately by the import refinement's PATCH_SAG_TOL contract).
	{
		let mut sag = w.max_sag;
		let mut edge_sag = |surface: &Surface, a: u32, b: u32| {
			let pair = if a < b { (a, b) } else { (b, a) };
			if !matches!(vert_pair_curve.get(&pair), Some(Curve::Circle { .. })) {
				let mid = 0.5 * (positions[a as usize] + positions[b as usize]);
				sag = sag.max(surface.unsigned_distance(mid));
			}
		};
		for fid in solid.faces() {
			let face = solid.face(fid);
			if solid.face_vertices(fid).len() < 3
				|| matches!(face.surface, Surface::Plane { .. })
				|| merged_away.contains(&fid.0)
				|| (!patches.is_empty() && covered(fid))
			{
				continue;
			}
			for lid in std::iter::once(face.outer).chain(face.inner.iter().copied()) {
				let verts: Vec<u32> = solid.loop_half_edges(lid).into_iter().map(|he| solid.half_edge(he).origin.0).collect();
				for k in 0..verts.len() {
					let (a, b) = (verts[k], verts[(k + 1) % verts.len()]);
					if a != b {
						edge_sag(&face.surface, a, b);
					}
				}
			}
		}
		for region in &regions {
			let surface = &solid.face(region.rep).surface;
			if matches!(surface, Surface::Plane { .. }) {
				continue; // a merged plane's boundary lies in the plane exactly
			}
			for ring in &region.loops {
				for k in 0..ring.len() {
					let (a, b) = (ring[k], ring[(k + 1) % ring.len()]);
					if a != b {
						edge_sag(surface, a, b);
					}
				}
			}
		}
		w.max_sag = sag;
	}

	// Build each face's geometry first, deferring the CLOSED_SHELL list.
	let mut advanced_faces: Vec<u32> = Vec::new();

	// One ORIENTED_EDGE walking vertex index a→b, creating/reusing the canonical
	// EDGE_CURVE (CIRCLE for circle-tagged pairs, LINE otherwise). `positions` and
	// `vertex_point` are passed per call (the patch ring loop grows them).
	let mut oriented_edge = |w: &mut StepWriter, positions: &[DVec3], vertex_point: &[u32], a: u32, b: u32| -> u32 {
		let key = if a < b { (a, b) } else { (b, a) };
		let edge_curve = match edge_curves.get(&key) {
			Some(&ec) => ec,
			None => {
				let va = vertex_point[key.0 as usize];
				let vb = vertex_point[key.1 as usize];
				// A circular edge (e.g. a cylinder rim) exports as a true CIRCLE arc;
				// every other edge is a straight LINE between its two vertices.
				let (geom, sense) = match vert_pair_curve.get(&key) {
					Some(Curve::Circle { center, normal, radius }) => {
						let nrm = normal.normalize_or_zero();
						let (xdir, ydir) = perp_basis(nrm);
						let placement = emit_placement(w, *center, nrm, xdir);
						// hash-consed: every arc of one rim references ONE CIRCLE
						let circle = w.emit_shared(&format!("CIRCLE('',#{placement},{})", real(*radius)));
						// The EDGE_CURVE's same_sense flag states whether walking
						// key.0→key.1 follows the circle's parameterisation. A rim
						// arc between adjacent ring vertices is the SHORT way
						// around, so the sign of the wrapped angle step decides;
						// an unconditional .T. would make a negative-step arc
						// re-import as its 2π-complement, sweeping the long way.
						let ang = |v: u32| {
							let d = positions[v as usize] - *center;
							d.dot(ydir).atan2(d.dot(xdir))
						};
						let mut step = ang(key.1) - ang(key.0);
						if step > std::f64::consts::PI {
							step -= std::f64::consts::TAU;
						} else if step <= -std::f64::consts::PI {
							step += std::f64::consts::TAU;
						}
						(circle, if step >= 0.0 { ".T." } else { ".F." })
					}
					_ => {
						let pa = positions[key.0 as usize];
						let pb = positions[key.1 as usize];
						let dir = (pb - pa).normalize_or_zero();
						let loc = w.emit_shared(&point("", pa));
						let vdir = w.emit_shared(&direction("", dir));
						let vector = w.emit_shared(&format!("VECTOR('',#{vdir},{})", real(1.0)));
						(w.emit_shared(&format!("LINE('',#{loc},#{vector})")), ".T.")
					}
				};
				let ec = w.emit(&format!("EDGE_CURVE('',#{va},#{vb},#{geom},{sense})"));
				edge_curves.insert(key, ec);
				ec
			}
		};
		// The ORIENTED_EDGE follows the loop direction a→b. The canonical
		// EDGE_CURVE runs key.0→key.1; if our loop goes b→a (a > b) the
		// orientation flag is .F.
		let orient_flag = if a == key.0 { ".T." } else { ".F." };
		w.emit(&format!("ORIENTED_EDGE('',*,*,#{edge_curve},{orient_flag})"))
	};

	for fid in solid.faces() {
		let face = solid.face(fid);
		if solid.face_vertices(fid).len() < 3 {
			// Degenerate outer loop: skip — it cannot form a valid FACE_OUTER_BOUND.
			continue;
		}
		if merged_away.contains(&fid.0) {
			continue; // replaced by a coalesced analytic face below
		}
		if !patches.is_empty() && covered(fid) {
			continue; // a freeform patch facet: replaced by the patch's ADVANCED_FACE
		}
		let centroid = face_centroid(solid, fid);

		// Geometric surface for this face.
		let surface_id = emit_surface(w, &face.surface, centroid);

		// One bound per loop: the outer loop as FACE_OUTER_BOUND, every inner (hole)
		// loop as FACE_BOUND — so a washer cap round-trips with its hole instead of
		// silently losing it.
		let mut bound_refs: Vec<u32> = Vec::new();
		for (li, lid) in std::iter::once(face.outer).chain(face.inner.iter().copied()).enumerate() {
			let verts: Vec<VertexId> = solid.loop_half_edges(lid).into_iter().map(|he| solid.half_edge(he).origin).collect();

			// Oriented edges around this loop.
			let n = verts.len();
			let mut oriented_edges: Vec<u32> = Vec::with_capacity(n);
			for k in 0..n {
				let a = verts[k].0;
				let b = verts[(k + 1) % n].0;
				if a == b {
					continue; // skip zero-length edge from a degenerate loop
				}
				oriented_edges.push(oriented_edge(w, &positions, &vertex_point, a, b));
			}

			if oriented_edges.len() < 3 {
				if li == 0 {
					bound_refs.clear();
					break; // degenerate outer loop: drop the whole face
				}
				continue; // degenerate inner sliver: drop just this hole
			}

			let refs = oriented_edges.iter().map(|id| format!("#{id}")).collect::<Vec<_>>().join(",");
			let edge_loop = w.emit(&format!("EDGE_LOOP('',({refs}))"));
			let bound = if li == 0 {
				w.emit(&format!("FACE_OUTER_BOUND('',#{edge_loop},.T.)"))
			} else {
				w.emit(&format!("FACE_BOUND('',#{edge_loop},.T.)"))
			};
			bound_refs.push(bound);
		}

		if bound_refs.is_empty() {
			continue; // no valid outer bound for this face
		}
		let bounds = bound_refs.iter().map(|id| format!("#{id}")).collect::<Vec<_>>().join(",");
		let af = w.emit(&format!("ADVANCED_FACE('',({bounds}),#{surface_id},.T.)"));
		advanced_faces.push(af);
	}

	// --- Coalesced analytic faces (one per merged same-surface region) --------
	for region in &regions {
		let face = solid.face(region.rep);
		let centroid = face_centroid(solid, region.rep);
		let surface_id = emit_surface(w, &face.surface, centroid);
		// outer loop = largest |projected signed area| (facet-inherited winding
		// keeps outers positive and holes negative in the surface frame)
		let basis = match face.surface {
			Surface::Plane { normal, .. } => perp_basis(normal.normalize_or_zero()),
			Surface::Cylinder { axis, .. } | Surface::Cone { axis, .. } => perp_basis(axis.normalize_or_zero()),
			_ => (DVec3::X, DVec3::Y),
		};
		let area2 = |ring: &Vec<u32>| -> f64 {
			let p2: Vec<(f64, f64)> = ring
				.iter()
				.map(|&vid| {
					let p = positions[vid as usize];
					(p.dot(basis.0), p.dot(basis.1))
				})
				.collect();
			let n = p2.len();
			(0..n)
				.map(|i| {
					let (x0, y0) = p2[i];
					let (x1, y1) = p2[(i + 1) % n];
					x0 * y1 - x1 * y0
				})
				.sum::<f64>()
				.abs()
		};
		let outer_idx = (0..region.loops.len()).max_by(|&i, &j| area2(&region.loops[i]).total_cmp(&area2(&region.loops[j]))).unwrap_or(0);
		let mut bound_refs: Vec<u32> = Vec::new();
		for (li, ring) in region.loops.iter().enumerate() {
			let n = ring.len();
			let mut oriented_edges: Vec<u32> = Vec::with_capacity(n);
			for k in 0..n {
				let (a, b) = (ring[k], ring[(k + 1) % n]);
				if a == b {
					continue;
				}
				oriented_edges.push(oriented_edge(w, &positions, &vertex_point, a, b));
			}
			if oriented_edges.len() < 3 {
				continue;
			}
			let refs = oriented_edges.iter().map(|id| format!("#{id}")).collect::<Vec<_>>().join(",");
			let edge_loop = w.emit(&format!("EDGE_LOOP('',({refs}))"));
			let bound = if li == outer_idx {
				w.emit(&format!("FACE_OUTER_BOUND('',#{edge_loop},.T.)"))
			} else {
				w.emit(&format!("FACE_BOUND('',#{edge_loop},.T.)"))
			};
			bound_refs.push(bound);
		}
		if bound_refs.is_empty() {
			continue;
		}
		let bounds = bound_refs.iter().map(|id| format!("#{id}")).collect::<Vec<_>>().join(",");
		let af = w.emit(&format!("ADVANCED_FACE('',({bounds}),#{surface_id},.T.)"));
		advanced_faces.push(af);
	}

	// --- One true B-spline ADVANCED_FACE per freeform patch -------------------
	// The trim rings are the recorded verbatim chords: their positions intern back
	// to solid vertex indices, so the LINE edges they create (or reuse) pair with
	// the neighbouring faceted faces' edges — including a slit ring's seam edge,
	// which appears twice (once per traversal direction) in ONE loop.
	for patch in patches {
		let surface_id = emit_bspline_surface(w, &patch.surface);
		let mut bound_refs: Vec<u32> = Vec::new();
		for (li, ring) in patch.rings.iter().enumerate() {
			let mut pts = ring.clone();
			pts.dedup_by(|a, b| key(*a) == key(*b));
			while pts.len() > 1 && key(pts[0]) == key(pts[pts.len() - 1]) {
				pts.pop();
			}
			let idx: Vec<u32> = pts
				.iter()
				.map(|&p| {
					*vert_index.entry(key(p)).or_insert_with(|| {
						// A ring chord position absent from the solid (possible only if
						// its facet degenerated away): synthesize a vertex point.
						let cp = w.emit(&point("", p));
						vertex_point.push(w.emit(&format!("VERTEX_POINT('',#{cp})")));
						positions.push(p);
						(vertex_point.len() - 1) as u32
					})
				})
				.collect();
			let n = idx.len();
			let mut oriented_edges: Vec<u32> = Vec::with_capacity(n);
			for k in 0..n {
				let (a, b) = (idx[k], idx[(k + 1) % n]);
				if a == b {
					continue;
				}
				oriented_edges.push(oriented_edge(w, &positions, &vertex_point, a, b));
			}
			if oriented_edges.len() < 3 {
				if li == 0 {
					bound_refs.clear();
					break; // degenerate outer ring: drop the whole patch face
				}
				continue;
			}
			let refs = oriented_edges.iter().map(|id| format!("#{id}")).collect::<Vec<_>>().join(",");
			let edge_loop = w.emit(&format!("EDGE_LOOP('',({refs}))"));
			let bound = if li == 0 {
				w.emit(&format!("FACE_OUTER_BOUND('',#{edge_loop},.T.)"))
			} else {
				w.emit(&format!("FACE_BOUND('',#{edge_loop},.T.)"))
			};
			bound_refs.push(bound);
		}
		if bound_refs.is_empty() {
			continue;
		}
		let bounds = bound_refs.iter().map(|id| format!("#{id}")).collect::<Vec<_>>().join(",");
		advanced_faces.push(w.emit(&format!("ADVANCED_FACE('',({bounds}),#{surface_id},.T.)")));
	}

	// --- Shell + solid ---------------------------------------------------------
	let shell_refs = advanced_faces.iter().map(|id| format!("#{id}")).collect::<Vec<_>>().join(",");
	let closed_shell = w.emit(&format!("CLOSED_SHELL('',({shell_refs}))"));
	let brep = w.emit(&format!("MANIFOLD_SOLID_BREP('{}',#{closed_shell})", escape(product_name)));
	// Disarm the geometry cache: entity sharing is scoped to ONE solid — the
	// next brep of an assembly (and the assembly's own placements) must not
	// reference records inside this MANIFOLD_SOLID_BREP's subgraph.
	w.geom_cache = None;
	brep
}

/// Emit the shared units + geometric representation context and return its id.
/// Must be called AFTER the file's breps so the accuracy is final (the
/// single-solid path already does; [`export_step_assembly`] pre-allocates the
/// id and defers the records — forward references are legal ISO-10303-21).
fn emit_geom_context(w: &mut StepWriter) -> u32 {
	let id = w.alloc();
	emit_geom_context_at(w, id);
	id
}

/// Emit the geometric-context records under a pre-allocated `id`.
/// `distance_accuracy_value` is HONEST: the worst chord sag measured while
/// emitting this file's solids (see the sag pass in [`emit_brep`]), floored at
/// the 1e-6 write precision — a solid whose curved boundaries are all true
/// arcs/rulings keeps 1e-6.
fn emit_geom_context_at(w: &mut StepWriter, id: u32) {
	let len_unit = w.emit("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )");
	let ang_unit = w.emit("( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) )");
	let solid_ang_unit = w.emit("( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() )");
	let uncertainty = w.emit(&format!(
		"UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE({}),#{len_unit},'distance_accuracy_value','confusion accuracy')",
		real(w.max_sag.max(1e-6))
	));
	w.emit_with(id, &format!(
		"( GEOMETRIC_REPRESENTATION_CONTEXT(3) GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{uncertainty})) GLOBAL_UNIT_ASSIGNED_CONTEXT((#{len_unit},#{ang_unit},#{solid_ang_unit})) REPRESENTATION_CONTEXT('','3D') )"
	));
}

/// Emit the `APPLICATION_CONTEXT` + `APPLICATION_PROTOCOL_DEFINITION` of `flavor`
/// and return the application-context id.
fn emit_app_context(w: &mut StepWriter, flavor: StepFlavor) -> u32 {
	match flavor {
		StepFlavor::Ap203 => {
			let app = w.emit("APPLICATION_CONTEXT('configuration controlled 3d designs of mechanical parts and assemblies')");
			w.emit(&format!("APPLICATION_PROTOCOL_DEFINITION('international standard','config_control_design',1994,#{app})"));
			app
		}
		StepFlavor::Ap242 => {
			let app = w.emit("APPLICATION_CONTEXT('managed model based 3d engineering')");
			w.emit(&format!(
				"APPLICATION_PROTOCOL_DEFINITION('international standard','ap242_managed_model_based_3d_engineering',2011,#{app})"
			));
			app
		}
	}
}

/// Emit one product (PRODUCT → … → PRODUCT_DEFINITION) under `app_context` and
/// return its `PRODUCT_DEFINITION` id. AP242 uses a plain
/// `PRODUCT_DEFINITION_FORMATION` and files the product under a
/// `PRODUCT_RELATED_PRODUCT_CATEGORY('part', …)`.
fn emit_product(w: &mut StepWriter, name: &str, flavor: StepFlavor, app_context: u32) -> u32 {
	let product_context = w.emit(&format!("PRODUCT_CONTEXT('',#{app_context},'mechanical')"));
	let product = w.emit(&format!("PRODUCT('{0}','{0}','',(#{product_context}))", escape(name)));
	let formation = match flavor {
		StepFlavor::Ap203 => w.emit(&format!("PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE('','',#{product},.NOT_KNOWN.)")),
		StepFlavor::Ap242 => {
			let f = w.emit(&format!("PRODUCT_DEFINITION_FORMATION('','',#{product})"));
			w.emit(&format!("PRODUCT_RELATED_PRODUCT_CATEGORY('part',$,(#{product}))"));
			f
		}
	};
	let pd_context = w.emit(&format!("PRODUCT_DEFINITION_CONTEXT('part definition',#{app_context},'design')"));
	w.emit(&format!("PRODUCT_DEFINITION('design','',#{formation},#{pd_context})"))
}

/// Wrap a DATA-section `body` in the ISO-10303-21 physical-file envelope of `flavor`.
fn assemble_file(body: &str, product_name: &str, flavor: StepFlavor) -> String {
	let (schema, ap) = match flavor {
		StepFlavor::Ap203 => ("CONFIG_CONTROL_DESIGN", "AP203"),
		StepFlavor::Ap242 => ("AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF { 1 0 10303 442 1 1 4 }", "AP242"),
	};
	let mut out = String::new();
	out.push_str("ISO-10303-21;\n");
	out.push_str("HEADER;\n");
	out.push_str(&format!("FILE_DESCRIPTION(('{}'),'2;1');\n", escape(&format!("STEP {ap} export of {product_name}"))));
	out.push_str(&format!(
		"FILE_NAME('{}','{}',(''),(''),'LMCAD kernel-brep','LMCAD','');\n",
		escape(product_name),
		// A fixed, deterministic timestamp placeholder keeps output stable.
		"1970-01-01T00:00:00"
	));
	out.push_str(&format!("FILE_SCHEMA(('{schema}'));\n"));
	out.push_str("ENDSEC;\n");
	out.push_str("DATA;\n");
	out.push_str(body);
	out.push_str("ENDSEC;\n");
	out.push_str("END-ISO-10303-21;\n");
	out
}

/// One-solid export shared by every flavor wrapper.
fn export_step_solid(solid: &Solid, patches: &[FreeformFace], product_name: &str, flavor: StepFlavor, coalesce: bool) -> String {
	let mut w = StepWriter::new();
	let brep = emit_brep(&mut w, solid, patches, product_name, coalesce);
	let geom_context = emit_geom_context(&mut w);
	let shape_rep = w.emit(&format!("ADVANCED_BREP_SHAPE_REPRESENTATION('{}',(#{brep}),#{geom_context})", escape(product_name)));
	let app_context = emit_app_context(&mut w, flavor);
	let product_def = emit_product(&mut w, product_name, flavor, app_context);
	let product_def_shape = w.emit(&format!("PRODUCT_DEFINITION_SHAPE('','',#{product_def})"));
	w.emit(&format!("SHAPE_DEFINITION_REPRESENTATION(#{product_def_shape},#{shape_rep})"));
	assemble_file(&w.body, product_name, flavor)
}

/// Export `solid` as an ISO-10303-21 (STEP AP203) physical file string.
///
/// `product_name` is used for the `PRODUCT` / `FILE_NAME` identifiers. The
/// returned string starts with `ISO-10303-21;` and ends with
/// `END-ISO-10303-21;`.
/// Coalesced output is SELF-VERIFIED: the merged-face body must round-trip
/// through our own importer to the same volume as the faceted solid. If it
/// does not (analytic reconstruction has known sharp edges around interrupted
/// bands and seam splits), that solid is exported fully faceted instead —
/// larger file, never silently wrong geometry.
fn coalesce_roundtrip_ok(solid: &Solid, product_name: &str) -> bool {
	if std::env::var_os("LMCAD_COAL_OFF").is_some() {
		return false;
	}
	let candidate = export_step_solid(solid, &[], product_name, StepFlavor::Ap203, true);
	let Ok(back) = crate::step_import::import_step(&candidate) else {
		return false;
	};
	let v0 = crate::validate::volume(solid).abs();
	let v1 = crate::validate::volume(&back).abs();
	v0 > 0.0 && ((v1 - v0).abs() / v0) < 0.005
}

pub fn export_step(solid: &Solid, product_name: &str) -> String {
	let co = coalesce_roundtrip_ok(solid, product_name);
	export_step_solid(solid, &[], product_name, StepFlavor::Ap203, co)
}

/// [`export_step`] with **AP242** product/header boilerplate (edition-1 file
/// schema, `'managed model based 3d engineering'` application protocol,
/// `PRODUCT_RELATED_PRODUCT_CATEGORY`). The geometry entities are identical to
/// the AP203 export — see the module's "AP242 honesty scope" for exactly what is
/// and is NOT claimed (no PMI/GD&T, no tessellated representations, no
/// kinematics, no validation properties).
pub fn export_step_ap242(solid: &Solid, product_name: &str) -> String {
	let co = coalesce_roundtrip_ok(solid, product_name);
	export_step_solid(solid, &[], product_name, StepFlavor::Ap242, co)
}

/// Export `solid` together with its **freeform (NURBS) sidecar**: every
/// [`FreeformFace`] (as returned by [`crate::import_step_freeform`]) is written as
/// ONE `ADVANCED_FACE` over a true `B_SPLINE_SURFACE_WITH_KNOTS` (rational
/// `_COMPLEX` when weighted), trimmed by its recorded rings as `LINE` polylines;
/// the solid's chord facets lying on a patch are skipped (they ARE that patch's
/// tessellation). This is the writing half of NURBS interchange: a trimmed-NURBS
/// STEP file re-imported with [`crate::import_step_freeform`] re-exports through
/// here with its B-spline surfaces intact rather than as facet soup.
pub fn export_step_freeform(solid: &Solid, patches: &[FreeformFace], product_name: &str) -> String {
	let co = coalesce_roundtrip_ok(solid, product_name);
	export_step_solid(solid, patches, product_name, StepFlavor::Ap203, co)
}

/// Export an **assembly**: `parts` are `(product name, part solid, placement)`
/// instances — exactly the triples [`crate::import_step_assembly`] returns, so an
/// imported assembly re-exports directly. Writes a root product plus one
/// `NEXT_ASSEMBLY_USAGE_OCCURRENCE` per instance, each placed by an
/// `ITEM_DEFINED_TRANSFORMATION` (identity frame → instance frame) inside a
/// `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`. Instances sharing a product name
/// share ONE product and ONE brep — the first such instance's geometry (STEP
/// products are identified by name).
///
/// Placements must be **rigid** (right-handed rotation + translation): an
/// `AXIS2_PLACEMENT_3D` cannot encode mirroring or scaling, so those are refused
/// loudly with [`StepError::Unsupported`] instead of silently re-orthogonalised.
pub fn export_step_assembly(parts: &[(String, Solid, DAffine3)], assembly_name: &str) -> Result<String, StepError> {
	use std::collections::HashMap;
	if parts.is_empty() {
		return Err(StepError::Topology("an assembly export needs at least one component instance".into()));
	}
	// Validate rigidity before writing anything.
	for (name, _, t) in parts {
		let m = t.matrix3;
		let ortho = m.x_axis.length() - 1.0;
		let (oy, oz) = (m.y_axis.length() - 1.0, m.z_axis.length() - 1.0);
		let skew = m.x_axis.dot(m.y_axis).abs().max(m.y_axis.dot(m.z_axis).abs()).max(m.z_axis.dot(m.x_axis).abs());
		let handed = m.x_axis.cross(m.y_axis).dot(m.z_axis);
		if ortho.abs() > 1e-9 || oy.abs() > 1e-9 || oz.abs() > 1e-9 || skew > 1e-9 || handed < 0.0 {
			return Err(StepError::Unsupported(format!(
				"assembly instance '{name}' has a non-rigid placement (scaled, skewed or mirrored) — AXIS2_PLACEMENT_3D cannot represent it"
			)));
		}
	}
	let mut w = StepWriter::new();
	// The context id is pre-allocated and its records emitted LAST (after every
	// part brep), so its honest distance_accuracy_value can aggregate the worst
	// chord sag across all parts. Forward id references are legal ISO-10303-21.
	let geom_context = w.alloc();
	let app_context = emit_app_context(&mut w, StepFlavor::Ap203);

	// Root product: a placement-only SHAPE_REPRESENTATION (no breps of its own).
	let root_pd = emit_product(&mut w, assembly_name, StepFlavor::Ap203, app_context);
	let root_pds = w.emit(&format!("PRODUCT_DEFINITION_SHAPE('','',#{root_pd})"));
	let origin = emit_placement(&mut w, DVec3::ZERO, DVec3::Z, DVec3::X);
	let root_rep = w.emit(&format!("SHAPE_REPRESENTATION('{}',(#{origin}),#{geom_context})", escape(assembly_name)));
	w.emit(&format!("SHAPE_DEFINITION_REPRESENTATION(#{root_pds},#{root_rep})"));

	// One product + brep per distinct part name (first instance's geometry wins).
	let mut part_ids: HashMap<&str, (u32, u32)> = HashMap::new(); // name → (product_def, shape_rep)
	for (name, solid, _) in parts {
		if part_ids.contains_key(name.as_str()) {
			continue;
		}
		let pd = emit_product(&mut w, name, StepFlavor::Ap203, app_context);
		let co = coalesce_roundtrip_ok(solid, name);
		let brep = emit_brep(&mut w, solid, &[], name, co);
		let rep = w.emit(&format!("ADVANCED_BREP_SHAPE_REPRESENTATION('{}',(#{brep}),#{geom_context})", escape(name)));
		let pds = w.emit(&format!("PRODUCT_DEFINITION_SHAPE('','',#{pd})"));
		w.emit(&format!("SHAPE_DEFINITION_REPRESENTATION(#{pds},#{rep})"));
		part_ids.insert(name.as_str(), (pd, rep));
	}

	// One NAUO + placed CONTEXT_DEPENDENT_SHAPE_REPRESENTATION per instance.
	for (k, (name, _, t)) in parts.iter().enumerate() {
		let (part_pd, part_rep) = part_ids[name.as_str()];
		let nauo = w.emit(&format!("NEXT_ASSEMBLY_USAGE_OCCURRENCE('NAUO{k}','{}','',#{root_pd},#{part_pd},$)", escape(name)));
		let pds = w.emit(&format!("PRODUCT_DEFINITION_SHAPE('placement','',#{nauo})"));
		let f1 = emit_placement(&mut w, DVec3::ZERO, DVec3::Z, DVec3::X);
		let f2 = emit_placement(&mut w, t.translation, t.matrix3.z_axis, t.matrix3.x_axis);
		let idt = w.emit(&format!("ITEM_DEFINED_TRANSFORMATION('','',#{f1},#{f2})"));
		let rel = w.emit(&format!(
			"( REPRESENTATION_RELATIONSHIP('','',#{part_rep},#{root_rep}) REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#{idt}) SHAPE_REPRESENTATION_RELATIONSHIP() )"
		));
		w.emit(&format!("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#{rel},#{pds})"));
	}
	emit_geom_context_at(&mut w, geom_context);
	Ok(assemble_file(&w.body, assembly_name, StepFlavor::Ap203))
}

/// Centroid of a face's outer-loop vertices.
fn face_centroid(solid: &Solid, fid: FaceId) -> DVec3 {
	let verts = solid.face_vertices(fid);
	if verts.is_empty() {
		return DVec3::ZERO;
	}
	let sum: DVec3 = verts.iter().map(|&v| solid.position(v)).sum();
	sum / verts.len() as f64
}

/// Escape a string for inclusion in a STEP single-quoted literal: single quotes
/// are doubled per ISO-10303-21. Other characters are passed through.
fn escape(s: &str) -> String {
	s.replace('\'', "''")
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::build::cuboid;
	use kernel_core::math::DVec3;

	#[test]
	fn cuboid_step_is_structurally_valid() {
		let solid = cuboid(DVec3::ZERO, DVec3::splat(1.0));
		let step = export_step(&solid, "test_box");

		// Envelope.
		assert!(step.starts_with("ISO-10303-21;"), "must start with magic header");
		assert!(step.trim_end().ends_with("END-ISO-10303-21;"), "must end with magic footer");

		// Required structural records.
		assert!(step.contains("MANIFOLD_SOLID_BREP"));
		assert!(step.contains("CLOSED_SHELL"));
		assert!(step.contains("FILE_SCHEMA(('CONFIG_CONTROL_DESIGN'))"));
		assert!(step.contains("ADVANCED_BREP_SHAPE_REPRESENTATION"));

		// Exactly 8 CARTESIAN_POINT lines belong to the 8 box vertices. The box
		// is all planar faces, so the only other CARTESIAN_POINTs are placement
		// locations and LINE origins. We assert specifically on the vertex
		// CARTESIAN_POINTs being 8 by counting VERTEX_POINT records (one per
		// vertex), and that the box has 8 vertices.
		let vertex_points = step.matches("VERTEX_POINT(").count();
		assert_eq!(vertex_points, 8, "cuboid has 8 vertices");
		assert_eq!(solid.vertex_count(), 8);

		// Every `#N =` entity id must be unique.
		use std::collections::HashSet;
		let mut ids = HashSet::new();
		for line in step.lines() {
			let line = line.trim_start();
			if let Some(rest) = line.strip_prefix('#') {
				if let Some(eq) = rest.find(" =") {
					let id = &rest[..eq];
					assert!(ids.insert(id.to_string()), "duplicate entity id #{id}");
				}
			}
		}
		// A cuboid yields many entities; at minimum the 8 vertex points exist.
		assert!(ids.len() >= 8);
	}

	#[test]
	fn cuboid_has_eight_vertex_cartesian_points() {
		// The 8 VERTEX_POINTs reference exactly 8 distinct vertex CARTESIAN_POINTs.
		// Each VERTEX_POINT('',#cp) references the CARTESIAN_POINT emitted
		// immediately before it. Collect those referenced ids and assert there
		// are 8 unique ones.
		let solid = cuboid(DVec3::new(-1.0, -2.0, -3.0), DVec3::new(4.0, 5.0, 6.0));
		let step = export_step(&solid, "box2");
		use std::collections::HashSet;
		let mut referenced = HashSet::new();
		for line in step.lines() {
			if let Some(start) = line.find("VERTEX_POINT('',#") {
				let tail = &line[start + "VERTEX_POINT('',#".len()..];
				let end = tail.find(')').unwrap();
				referenced.insert(tail[..end].to_string());
			}
		}
		assert_eq!(referenced.len(), 8, "8 unique vertex CARTESIAN_POINT refs");
	}

	#[test]
	fn real_always_has_decimal_point() {
		assert!(real(1.0).contains('.'));
		assert!(real(0.0).contains('.'));
		assert!(real(-2.5).contains('.'));
		// Non-finite is clamped to a finite literal.
		assert!(real(f64::NAN).contains('.'));
	}

	#[test]
	fn ap242_export_carries_the_ap242_envelope_and_reimports() {
		use crate::build::cylinder;
		use crate::import_step;
		use crate::validate::{validate, volume};
		// The AP242 flavor differs from AP203 ONLY in the envelope (file schema,
		// application context/protocol, formation + product category) — the module
		// docs state the honesty scope (no PMI/GD&T/tessellated reps/kinematics).
		// The geometry must re-import identically.
		let cyl = cylinder(DVec3::ZERO, DVec3::Z, 2.0, 5.0, 16);
		let step = export_step_ap242(&cyl, "cyl242");
		let back = import_step(&step).expect("the AP242 export must re-import");
		let v = validate(&back);
		assert!(
			step.contains("FILE_SCHEMA(('AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF { 1 0 10303 442 1 1 4 }'))")
				&& step.contains("APPLICATION_CONTEXT('managed model based 3d engineering')")
				&& step.contains("'ap242_managed_model_based_3d_engineering'")
				&& step.contains("PRODUCT_RELATED_PRODUCT_CATEGORY('part'")
				&& !step.contains("CONFIG_CONTROL_DESIGN")
				&& v.is_valid()
				&& (volume(&back) - volume(&cyl)).abs() < 1e-6,
			"AP242 flavor must swap the envelope and keep the geometry: validity {v:?}, vol {} vs {}",
			volume(&back),
			volume(&cyl)
		);
	}

	#[test]
	fn cylinder_rims_export_as_circle_arcs() {
		use crate::build::cylinder;
		// The cylinder's rim edges carry an analytic Curve::Circle, so every rim
		// EDGE_CURVE references a true CIRCLE (faithful round geometry) rather than
		// a straight LINE chord. Entity dedup shares ONE CIRCLE record per rim
		// (equal parameters + placement), so a 12-segment cylinder emits exactly
		// 2 CIRCLE entities, referenced by all 24 rim-arc EDGE_CURVEs. A box,
		// with no circular edges, emits none.
		let step = export_step(&cylinder(DVec3::ZERO, DVec3::Z, 2.0, 5.0, 12), "cyl");
		let circle_ids: std::collections::HashSet<&str> = step
			.lines()
			.filter(|l| l.contains("= CIRCLE('',#"))
			.filter_map(|l| l.split(" =").next())
			.map(|id| id.trim_start_matches('#'))
			.collect();
		// EDGE_CURVE('',#va,#vb,#geom,<sense>) — field 3 after splitting on ",#".
		let arc_edges = step
			.lines()
			.filter(|l| l.contains("= EDGE_CURVE("))
			.filter(|l| l.split(",#").nth(3).and_then(|t| t.split(',').next()).is_some_and(|geom| circle_ids.contains(geom)))
			.count();
		assert!(
			circle_ids.len() == 2 && arc_edges == 24,
			"12-seg cylinder: {} CIRCLE entities (want 2 — one per rim, deduped) referenced by {arc_edges} rim-arc EDGE_CURVEs (want 24)",
			circle_ids.len()
		);
		assert!(step.contains("CYLINDRICAL_SURFACE"), "the lateral face is a cylindrical surface");
		assert!(!export_step(&cuboid(DVec3::ZERO, DVec3::splat(1.0)), "box").contains("CIRCLE('',#"), "a box has no circular edges");
	}
}
