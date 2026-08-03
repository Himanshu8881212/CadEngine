// Copyright (c) LMCAD. Licensed under the MIT License.

//! Index-arena half-edge (DCEL) topology.
//!
//! Per the spec's Rust guidance: arenas (`Vec` + newtype handles), **not**
//! `Rc<RefCell>`. The hierarchy is `Solid → Shell → Face → Loop → HalfEdge →
//! Edge → Vertex`. A half-edge gives O(1) adjacency traversal and is the
//! simplest topology that correctly represents orientable manifold solids.

use kernel_core::math::DVec3;

use crate::geom::{Curve, Surface};

/// Whether two analytic surfaces are the same (variant + parameters within tolerance) —
/// used to collect the *distinct* curved surfaces of a faceted solid for sectioning.
fn surfaces_eq(a: &Surface, b: &Surface) -> bool {
	let close = |x: f64, y: f64| (x - y).abs() < 1e-9;
	let cv = |u: DVec3, v: DVec3| (u - v).length() < 1e-9;
	match (a, b) {
		(Surface::Plane { origin: o1, normal: n1 }, Surface::Plane { origin: o2, normal: n2 }) => cv(*o1, *o2) && cv(*n1, *n2),
		(Surface::Cylinder { origin: o1, axis: a1, radius: r1 }, Surface::Cylinder { origin: o2, axis: a2, radius: r2 }) => {
			cv(*o1, *o2) && cv(*a1, *a2) && close(*r1, *r2)
		}
		(Surface::Sphere { center: c1, radius: r1 }, Surface::Sphere { center: c2, radius: r2 }) => cv(*c1, *c2) && close(*r1, *r2),
		(Surface::Cone { apex: p1, axis: a1, half_angle: h1 }, Surface::Cone { apex: p2, axis: a2, half_angle: h2 }) => {
			cv(*p1, *p2) && cv(*a1, *a2) && close(*h1, *h2)
		}
		(Surface::Torus { center: c1, axis: a1, major: m1, minor: n1 }, Surface::Torus { center: c2, axis: a2, major: m2, minor: n2 }) => {
			cv(*c1, *c2) && cv(*a1, *a2) && close(*m1, *m2) && close(*n1, *n2)
		}
		_ => false,
	}
}

/// Whether a sampled point of `curve` lies within the (slightly expanded) box `[lo, hi]` —
/// a cheap filter to drop surface-section curves that miss the solid.
fn curve_touches_box(curve: &Curve, lo: DVec3, hi: DVec3) -> bool {
	let inb = |p: DVec3| (lo.x - 1e-6..=hi.x + 1e-6).contains(&p.x) && (lo.y - 1e-6..=hi.y + 1e-6).contains(&p.y) && (lo.z - 1e-6..=hi.z + 1e-6).contains(&p.z);
	let span = (hi - lo).length().max(1.0);
	(0..16).any(|k| {
		let t = match curve {
			// Closed conics sweep an angle; an open line/branch samples a length range.
			Curve::Circle { .. } | Curve::Ellipse { .. } => std::f64::consts::TAU * k as f64 / 16.0,
			_ => span * (k as f64 / 15.0 - 0.5) * 2.0,
		};
		inb(curve.point_at(t))
	})
}

macro_rules! handle {
	($name:ident) => {
		#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, PartialOrd, Ord)]
		pub struct $name(pub u32);
		impl $name {
			#[inline]
			fn ix(self) -> usize {
				self.0 as usize
			}
		}
	};
}

handle!(VertexId);
handle!(HalfEdgeId);
handle!(EdgeId);
handle!(LoopId);
handle!(FaceId);
handle!(ShellId);

#[derive(Clone, Copy, Debug)]
pub struct Vertex {
	pub position: DVec3,
	/// One half-edge originating at this vertex.
	pub half_edge: HalfEdgeId,
}

#[derive(Clone, Copy, Debug)]
pub struct HalfEdge {
	pub origin: VertexId,
	pub twin: Option<HalfEdgeId>,
	pub next: HalfEdgeId,
	pub prev: HalfEdgeId,
	pub face: FaceId,
	pub edge: EdgeId,
	pub loop_id: LoopId,
}

#[derive(Clone, Copy, Debug)]
pub struct Edge {
	/// One of the two half-edges sharing this edge.
	pub half_edge: HalfEdgeId,
	pub curve: Option<Curve>,
}

#[derive(Clone, Debug)]
pub struct Loop {
	pub first: HalfEdgeId,
	pub is_outer: bool,
	pub face: FaceId,
}

#[derive(Clone, Debug)]
pub struct Face {
	pub outer: LoopId,
	pub inner: Vec<LoopId>,
	pub surface: Surface,
	pub shell: ShellId,
}

#[derive(Clone, Debug)]
pub struct Shell {
	pub faces: Vec<FaceId>,
	pub is_closed: bool,
}

/// One boundary loop of a face plus the analytic surface it lies on.
pub struct FaceInput {
	/// Vertex indices in CCW order as seen from outside the solid.
	pub boundary: Vec<u32>,
	pub surface: Surface,
}

/// A face with one or more boundary loops on an analytic surface — `loops[0]` is the
/// outer loop (CCW seen from outside) and `loops[1..]` are inner hole loops (wound the
/// opposite way). Lets a face carry a hole (a washer's cap) or, in future, two rim loops
/// of a periodic surface — the topology a single analytic curved face needs.
pub struct FaceLoops {
	pub loops: Vec<Vec<u32>>,
	pub surface: Surface,
}

/// Which operand of a boolean a result face came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum FaceSource {
	/// A face of a construction primitive (box, cylinder, …), named directly by
	/// its canonical face index. Lets a primitive's faces and edges carry stable
	/// names so a feature (e.g. a fillet) can be put on "that edge" before any
	/// boolean — and have the name re-resolve after a parameter edit. A primitive
	/// name is re-tagged by the immediate operand when it enters a boolean, so it
	/// never leaks into boolean-result provenance.
	Primitive,
	/// A face lying on (a surface of) the first boolean operand `A`.
	OperandA,
	/// A face lying on (a surface of) the second boolean operand `B`.
	OperandB,
}

/// A stable, persistent **name** for a boolean-result face: the operand it came
/// from and the index of the *original face of that operand* it lies on. Because
/// the name refers to the input topology (not the freshly-generated result
/// indices), it survives re-evaluation — a feature can store a `FaceName` and
/// re-select the corresponding result face after an upstream parameter edit
/// (the foundation of topological naming / persistent feature references).
///
/// A single original face may be split into several result faces, so a name can
/// resolve to more than one [`FaceId`] (see [`Solid::faces_named`]).
///
/// # Merged-face name policy (rebuild passes)
///
/// The rebuild passes that merge faces — [`crate::coalesce_coplanar`] and
/// [`crate::recover::recover_quadrics`] — carry provenance through the rebuild
/// under ONE deterministic rule: an **unmerged face keeps its name exactly**,
/// and a **merged face inherits the lexicographically-least constituent
/// `FaceName`** (the derived `Ord`: operand `Primitive < OperandA < OperandB`,
/// then `source_face`) — stable across runs because it depends only on the
/// constituent names, never on traversal order. Consequences, stated: the
/// non-least names of a multi-name merge (fragments of several source faces
/// flush-merged into one) and the names of fully-consumed interior fragments
/// stop resolving — they name faces that no longer exist. [`EdgeName`]s and
/// [`VertexName`]s, being derived from face names + topology, re-resolve
/// automatically wherever their faces survived; a fragmented edge whose
/// collinear pieces all merged back re-resolves to ONE edge again (the
/// FRICTION #20 witness-re-resolution fix).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct FaceName {
	/// The boolean operand this face came from.
	pub operand: FaceSource,
	/// The index of the operand's original face this result face lies on.
	pub source_face: u32,
}

/// A stable, persistent **name** for a boolean-result edge: the (canonically
/// ordered) pair of [`FaceName`]s of the two faces it bounds. An edge is the
/// intersection of two faces, so naming it by its two named faces gives a
/// reference that survives re-evaluation just like [`FaceName`] does — the handle a
/// feature needs to put a fillet or chamfer on "that edge" and have it stick.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct EdgeName {
	/// The two faces meeting at the edge, ordered so the name is independent of
	/// which side is listed first.
	pub faces: [FaceName; 2],
}

impl EdgeName {
	/// Canonical edge name from the two adjacent face names.
	pub fn new(a: FaceName, b: FaceName) -> Self {
		if a <= b {
			EdgeName { faces: [a, b] }
		} else {
			EdgeName { faces: [b, a] }
		}
	}
}

/// A stable, persistent **name** for a boolean-result vertex: the (canonically
/// ordered) triple of [`FaceName`]s of the three faces meeting at a trivalent corner.
/// Like [`EdgeName`] it is derived purely from face provenance + topology, so it
/// survives re-evaluation — the handle a feature needs to reference "that corner"
/// (e.g. to place a vertex blend) and have it stick across a parameter edit.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct VertexName {
	/// The three faces meeting at the corner, ordered so the name is independent of
	/// which is listed first.
	pub faces: [FaceName; 3],
}

impl VertexName {
	/// Canonical vertex name from the three corner face names.
	pub fn new(a: FaceName, b: FaceName, c: FaceName) -> Self {
		let mut faces = [a, b, c];
		faces.sort();
		VertexName { faces }
	}
}

/// A solid body: the arena owning all topology and geometry.
#[derive(Clone, Debug, Default)]
pub struct Solid {
	pub(crate) vertices: Vec<Vertex>,
	pub(crate) half_edges: Vec<HalfEdge>,
	pub(crate) edges: Vec<Edge>,
	pub(crate) loops: Vec<Loop>,
	pub(crate) faces: Vec<Face>,
	pub(crate) shells: Vec<Shell>,
	/// Per-face name, parallel to `faces` (empty unless populated by a boolean).
	pub(crate) provenance: Vec<FaceName>,
}

impl Solid {
	// --- Accessors -----------------------------------------------------------

	pub fn vertex(&self, id: VertexId) -> &Vertex {
		&self.vertices[id.ix()]
	}
	pub fn half_edge(&self, id: HalfEdgeId) -> &HalfEdge {
		&self.half_edges[id.ix()]
	}
	pub fn edge(&self, id: EdgeId) -> &Edge {
		&self.edges[id.ix()]
	}
	pub fn loop_(&self, id: LoopId) -> &Loop {
		&self.loops[id.ix()]
	}
	pub fn face(&self, id: FaceId) -> &Face {
		&self.faces[id.ix()]
	}
	pub fn shell(&self, id: ShellId) -> &Shell {
		&self.shells[id.ix()]
	}
	pub fn position(&self, id: VertexId) -> DVec3 {
		self.vertices[id.ix()].position
	}

	pub fn vertex_count(&self) -> usize {
		self.vertices.len()
	}
	pub fn half_edge_count(&self) -> usize {
		self.half_edges.len()
	}
	pub fn edge_count(&self) -> usize {
		self.edges.len()
	}
	pub fn face_count(&self) -> usize {
		self.faces.len()
	}
	pub fn shell_count(&self) -> usize {
		self.shells.len()
	}

	pub fn faces(&self) -> impl Iterator<Item = FaceId> {
		(0..self.faces.len() as u32).map(FaceId)
	}

	pub fn edges(&self) -> impl Iterator<Item = EdgeId> + '_ {
		(0..self.edges.len() as u32).map(EdgeId)
	}

	/// The boolean operand face `id` came from, or `None` when this solid did not
	/// record provenance (e.g. a primitive, not a boolean result).
	pub fn face_source(&self, id: FaceId) -> Option<FaceSource> {
		self.provenance.get(id.0 as usize).map(|n| n.operand)
	}

	/// The persistent [`FaceName`] of face `id` (operand + original face index), or
	/// `None` for a non-boolean solid. Store this to re-select the face after an edit.
	pub fn face_name(&self, id: FaceId) -> Option<FaceName> {
		self.provenance.get(id.0 as usize).copied()
	}

	/// Re-resolve a stored [`FaceName`] to the current result faces bearing it — the
	/// query a parametric feature uses to rebind "that face" after re-evaluation.
	/// Returns every matching [`FaceId`] (an original face may split into several).
	pub fn faces_named(&self, name: FaceName) -> Vec<FaceId> {
		self.provenance
			.iter()
			.enumerate()
			.filter(|(_, n)| **n == name)
			.map(|(i, _)| FaceId(i as u32))
			.collect()
	}

	/// The persistent [`EdgeName`] of edge `id` — the pair of [`FaceName`]s of the two
	/// faces it bounds — or `None` when either neighbour is unnamed (a non-boolean
	/// solid) or the edge is a boundary. Derived from the face provenance + topology,
	/// so it needs no separate edge-provenance bookkeeping.
	pub fn edge_name(&self, id: EdgeId) -> Option<EdgeName> {
		let edge = self.edges.get(id.ix())?;
		let he1 = self.half_edges.get(edge.half_edge.ix())?;
		let twin = he1.twin?;
		let a = self.face_name(he1.face)?;
		let b = self.face_name(self.half_edges.get(twin.ix())?.face)?;
		Some(EdgeName::new(a, b))
	}

	/// Re-resolve a stored [`EdgeName`] to the current edges bearing it — the query a
	/// feature uses to rebind a fillet/chamfer edge after a parametric edit.
	pub fn edges_named(&self, name: EdgeName) -> Vec<EdgeId> {
		(0..self.edges.len() as u32).map(EdgeId).filter(|&e| self.edge_name(e) == Some(name)).collect()
	}

	/// The persistent [`VertexName`] of vertex `id` — the triple of [`FaceName`]s of the
	/// three faces meeting there — or `None` unless it is a trivalent corner whose three
	/// faces are all named. Derived from face provenance + topology, like [`Self::edge_name`].
	pub fn vertex_name(&self, id: VertexId) -> Option<VertexName> {
		let mut face_ids: Vec<FaceId> = Vec::new();
		for he in &self.half_edges {
			if he.origin == id && !face_ids.contains(&he.face) {
				face_ids.push(he.face);
			}
		}
		if face_ids.len() != 3 {
			return None; // only trivalent corners are named
		}
		Some(VertexName::new(self.face_name(face_ids[0])?, self.face_name(face_ids[1])?, self.face_name(face_ids[2])?))
	}

	/// Re-resolve a stored [`VertexName`] to the current vertices bearing it — the query a
	/// feature uses to rebind a corner reference after a parametric edit.
	pub fn vertices_named(&self, name: VertexName) -> Vec<VertexId> {
		(0..self.vertices.len() as u32).map(VertexId).filter(|&v| self.vertex_name(v) == Some(name)).collect()
	}

	/// Exact analytic cross-section: the intersection [`Curve`]s of the plane through
	/// `plane_origin` with normal `plane_normal` and this solid's faces.
	///
	/// Each planar face the plane *crosses* contributes the section [`Curve::Line`]; each
	/// distinct curved surface contributes its closed-form section (a cylinder cut ⟂ its
	/// axis → a [`Curve::Circle`], obliquely → a [`Curve::Ellipse`]; a sphere → a circle;
	/// a cone → the conic), via [`Surface::plane_section`]. Curved sections are kept only
	/// when they pass through the solid's bounding box (a cheap containment filter). This
	/// is an exact query — no meshing — so an AI can read the true cross-section geometry.
	pub fn section_curves(&self, plane_origin: DVec3, plane_normal: DVec3) -> Vec<Curve> {
		let pn = plane_normal.normalize_or_zero();
		if pn.length_squared() < 0.5 {
			return Vec::new();
		}
		let (mut lo, mut hi) = (DVec3::splat(f64::INFINITY), DVec3::splat(f64::NEG_INFINITY));
		for v in &self.vertices {
			lo = lo.min(v.position);
			hi = hi.max(v.position);
		}
		let mut out: Vec<Curve> = Vec::new();
		let mut seen: Vec<Surface> = Vec::new();
		for f in self.faces() {
			let surf = self.face(f).surface;
			if let Surface::Plane { .. } = surf {
				// Include the section line only when the plane actually crosses this face
				// (its vertices straddle the plane), so parallel/non-cut faces don't appear.
				let sides: Vec<f64> = self.face_polygon(f).iter().map(|p| (*p - plane_origin).dot(pn)).collect();
				if sides.iter().any(|&s| s > 1e-9) && sides.iter().any(|&s| s < -1e-9) {
					out.extend(surf.plane_section(plane_origin, pn));
				}
			} else {
				// One analytic curve per distinct curved surface, kept if it touches the box.
				if seen.iter().any(|s| surfaces_eq(s, &surf)) {
					continue;
				}
				seen.push(surf);
				for c in surf.plane_section(plane_origin, pn) {
					if curve_touches_box(&c, lo, hi) {
						out.push(c);
					}
				}
			}
		}
		out
	}

	/// Half-edges of a loop in order, starting from the loop's first.
	pub fn loop_half_edges(&self, lp: LoopId) -> Vec<HalfEdgeId> {
		let start = self.loops[lp.ix()].first;
		let mut out = vec![start];
		let mut he = self.half_edges[start.ix()].next;
		while he != start {
			out.push(he);
			he = self.half_edges[he.ix()].next;
		}
		out
	}

	/// Vertex ids around a face's outer loop.
	pub fn face_vertices(&self, f: FaceId) -> Vec<VertexId> {
		let outer = self.faces[f.ix()].outer;
		self.loop_half_edges(outer)
			.into_iter()
			.map(|he| self.half_edges[he.ix()].origin)
			.collect()
	}

	/// Vertex positions around a face's outer loop.
	pub fn face_polygon(&self, f: FaceId) -> Vec<DVec3> {
		self.face_vertices(f).into_iter().map(|v| self.position(v)).collect()
	}

	/// Vertex positions around a single loop (a face's outer loop *or* one of its
	/// inner hole loops, from [`Face::inner`]).
	pub fn loop_polygon(&self, lid: LoopId) -> Vec<DVec3> {
		self.loop_half_edges(lid)
			.into_iter()
			.map(|he| self.position(self.half_edges[he.ix()].origin))
			.collect()
	}

	// --- Construction --------------------------------------------------------

	/// Build a solid from a vertex list and per-face boundary loops (one outer
	/// loop each). Twins are matched by reversed directed edges, so a closed
	/// manifold gets every half-edge paired and one [`Edge`] per pair.
	pub fn from_faces(positions: Vec<DVec3>, faces: Vec<FaceInput>) -> Solid {
		Self::from_faces_multiloop(
			positions,
			faces.into_iter().map(|f| FaceLoops { loops: vec![f.boundary], surface: f.surface }).collect(),
		)
	}

	/// Build a solid where each face may carry MULTIPLE boundary loops (an outer loop plus
	/// inner hole loops). Generalises [`Self::from_faces`] (which is the single-loop case);
	/// twin-matching by reversed directed edge is unchanged, so a hole's loop pairs with the
	/// surrounding face just like any other shared edge. The prerequisite for faces with
	/// holes (a washer cap) and, later, periodic curved faces.
	pub fn from_faces_multiloop(positions: Vec<DVec3>, faces: Vec<FaceLoops>) -> Solid {
		let mut solid = Solid::default();
		let shell_id = ShellId(0);

		// Vertices (half_edge patched in later).
		solid.vertices = positions
			.iter()
			.map(|&p| Vertex { position: p, half_edge: HalfEdgeId(0) })
			.collect();

		// Directed-edge → half-edge map for twin matching.
		use std::collections::HashMap;
		let mut dir_map: HashMap<(u32, u32), HalfEdgeId> = HashMap::new();
		let mut shell_faces = Vec::with_capacity(faces.len());

		for (fi, face_in) in faces.iter().enumerate() {
			let face_id = FaceId(fi as u32);
			let mut loop_ids: Vec<LoopId> = Vec::with_capacity(face_in.loops.len());
			for (li, lp) in face_in.loops.iter().enumerate() {
				let n = lp.len();
				assert!(n >= 3, "face {fi} loop {li} has fewer than 3 vertices");
				let loop_id = LoopId(solid.loops.len() as u32);
				let base = solid.half_edges.len() as u32;
				for k in 0..n {
					let origin = VertexId(lp[k]);
					let next = HalfEdgeId(base + ((k + 1) % n) as u32);
					let prev = HalfEdgeId(base + ((k + n - 1) % n) as u32);
					let he_id = HalfEdgeId(base + k as u32);
					solid.half_edges.push(HalfEdge {
						origin,
						twin: None,
						next,
						prev,
						face: face_id,
						edge: EdgeId(u32::MAX), // patched below
						loop_id,
					});
					// Record this directed edge for twin lookup. Keep the FIRST half-edge
					// for a given directed edge: a duplicate means the input is non-manifold
					// along (a,b) (e.g. a degenerate boolean of edge-touching solids).
					// Rather than panic, leave the extra half-edge unpaired so `validate`
					// reports it as a boundary instead of twin pairing being corrupted.
					let a = lp[k];
					let b = lp[(k + 1) % n];
					dir_map.entry((a, b)).or_insert(he_id);
					// Give the origin vertex an outgoing half-edge.
					solid.vertices[a as usize].half_edge = he_id;
				}
				solid.loops.push(Loop { first: HalfEdgeId(base), is_outer: li == 0, face: face_id });
				loop_ids.push(loop_id);
			}

			solid.faces.push(Face {
				outer: loop_ids[0],
				inner: loop_ids[1..].to_vec(),
				surface: face_in.surface,
				shell: shell_id,
			});
			shell_faces.push(face_id);
		}

		// Match twins and build one edge per undirected pair.
		for he_id in 0..solid.half_edges.len() as u32 {
			let he = solid.half_edges[he_id as usize];
			if he.edge != EdgeId(u32::MAX) {
				continue; // already assigned via its twin
			}
			let a = he.origin.0;
			let b = solid.half_edges[he.next.ix()].origin.0;
			let edge_id = EdgeId(solid.edges.len() as u32);
			solid.edges.push(Edge { half_edge: HalfEdgeId(he_id), curve: None });
			solid.half_edges[he_id as usize].edge = edge_id;
			if let Some(&twin) = dir_map.get(&(b, a)) {
				// Only pair when the candidate twin is itself still unpaired: for a
				// manifold input every reversed edge is unique so this always holds,
				// but a non-manifold input (coincident faces) would otherwise let two
				// half-edges claim the same twin and corrupt the pairing. The extra
				// half-edge is instead left unpaired (a boundary), never mis-twinned.
				if twin.ix() != he_id as usize && solid.half_edges[twin.ix()].twin.is_none() {
					solid.half_edges[he_id as usize].twin = Some(twin);
					solid.half_edges[twin.ix()].twin = Some(HalfEdgeId(he_id));
					solid.half_edges[twin.ix()].edge = edge_id;
				}
			}
		}

		let is_closed = solid.half_edges.iter().all(|he| he.twin.is_some());
		solid.shells.push(Shell { faces: shell_faces, is_closed });
		solid
	}

	/// Name every face by its canonical index as a construction primitive
	/// ([`FaceSource::Primitive`]). Because a primitive constructor always emits its
	/// faces in the same order, these names are stable across a parameter edit, so a
	/// box edge gets an [`EdgeName`] that re-resolves after the box is resized — the
	/// handle a fillet/chamfer feature needs.
	pub fn with_primitive_names(mut self) -> Self {
		self.provenance = (0..self.faces.len() as u32)
			.map(|k| FaceName { operand: FaceSource::Primitive, source_face: k })
			.collect();
		self
	}

	/// Overwrite the per-face provenance (parallel to `faces`). Used by operations
	/// that rebuild a solid and need to carry names onto the result faces.
	pub(crate) fn set_provenance(&mut self, provenance: Vec<FaceName>) {
		self.provenance = provenance;
	}

	/// Attach an analytic [`Curve`] to the edge between vertices `a` and `b` (in either
	/// order), returning whether such an edge was found. Lets a constructor record the
	/// exact geometry of a curved edge (e.g. a cylinder's circular rim) so it is not just
	/// a polyline — the basis for exact section queries and faithful STEP export.
	pub fn set_edge_curve(&mut self, a: VertexId, b: VertexId, curve: Curve) -> bool {
		for e in 0..self.edges.len() {
			let he = self.half_edges[self.edges[e].half_edge.ix()];
			let v0 = he.origin;
			let v1 = self.half_edges[he.next.ix()].origin;
			if (v0 == a && v1 == b) || (v0 == b && v1 == a) {
				self.edges[e].curve = Some(curve);
				return true;
			}
		}
		false
	}

	/// The analytic [`Curve`] attached to edge `id`, if any.
	pub fn edge_curve(&self, id: EdgeId) -> Option<Curve> {
		self.edges.get(id.ix()).and_then(|e| e.curve)
	}

	/// Set the analytic [`Curve`] of a specific edge by id (used by the boolean to tag a
	/// curved seam edge with its exact plane∩surface section).
	pub(crate) fn set_edge_curve_by_id(&mut self, id: EdgeId, curve: Curve) {
		if let Some(e) = self.edges.get_mut(id.ix()) {
			e.curve = Some(curve);
		}
	}

	/// Transform every vertex and surface by `m` (rigid + uniform scale).
	pub fn transformed(&self, m: kernel_core::math::DAffine3) -> Solid {
		let mut out = self.clone();
		for v in out.vertices.iter_mut() {
			v.position = m.transform_point3(v.position);
		}
		for f in out.faces.iter_mut() {
			f.surface = f.surface.transformed(m);
		}
		for e in out.edges.iter_mut() {
			if let Some(c) = e.curve {
				e.curve = Some(c.transformed(m));
			}
		}
		out
	}

	/// Reflect across the plane through `plane_point` with normal `plane_normal`,
	/// returning a correctly-oriented mirror copy.
	///
	/// A pure reflection reverses handedness, so simply transforming the vertices
	/// would leave the solid inside-out (inward normals, negative volume). This
	/// rebuilds the topology from the reflected vertices with **every loop** of each
	/// face reversed, which flips the winding back to outward — yielding a valid solid.
	/// Inner hole loops are carried (each reversed too), so mirroring a part with a
	/// pocket or bore preserves the hole instead of silently filling it.
	pub fn mirrored(&self, plane_point: DVec3, plane_normal: DVec3) -> Solid {
		let n = plane_normal.normalize_or_zero();
		if n.length_squared() < 0.5 {
			return self.clone();
		}
		// Reflection x ↦ x − 2((x−p)·n)n, as an affine for the analytic surfaces.
		let col = |e: DVec3, nj: f64| e - n * (2.0 * nj);
		let m3 = kernel_core::math::DMat3::from_cols(col(DVec3::X, n.x), col(DVec3::Y, n.y), col(DVec3::Z, n.z));
		let m = kernel_core::math::DAffine3::from_mat3_translation(m3, n * (2.0 * plane_point.dot(n)));

		let positions: Vec<DVec3> = (0..self.vertex_count() as u32)
			.map(|i| m.transform_point3(self.vertex(VertexId(i)).position))
			.collect();
		let loop_verts_reversed = |lp: LoopId| {
			let mut vs: Vec<u32> = self.loop_half_edges(lp).into_iter().map(|he| self.half_edges[he.ix()].origin.0).collect();
			vs.reverse();
			vs
		};
		let faces: Vec<FaceLoops> = self
			.faces()
			.map(|f| {
				let face = &self.faces[f.ix()];
				let loops: Vec<Vec<u32>> = std::iter::once(face.outer)
					.chain(face.inner.iter().copied())
					.map(loop_verts_reversed)
					.collect();
				FaceLoops { loops, surface: face.surface.transformed(m) }
			})
			.collect();
		Solid::from_faces_multiloop(positions, faces)
	}

	/// Axis-aligned bound of all vertices as `(min, max)` in f64. Returns an inverted
	/// (`+∞`, `−∞`) box for a vertex-less solid, so an overlap test reads as "no overlap".
	pub fn aabb(&self) -> (DVec3, DVec3) {
		let mut min = DVec3::splat(f64::INFINITY);
		let mut max = DVec3::splat(f64::NEG_INFINITY);
		for v in &self.vertices {
			min = min.min(v.position);
			max = max.max(v.position);
		}
		(min, max)
	}

	/// Combine two solids into ONE multi-shell solid by concatenating their topology, with NO
	/// boolean co-refinement. Correct (and EXACT) only when the two solids are geometrically
	/// **disjoint** (non-overlapping) — each becomes a separate shell. This is the precise way to
	/// place several non-touching parts (a bolt-circle's hole pattern, a tray of pegs) into one
	/// solid without the curved mesh-arrangement boolean that a chained `union` corrupts. (If the
	/// inputs actually overlap, use a real boolean [`crate::union`] instead — this does not fuse.)
	pub fn disjoint_union(&self, other: &Solid) -> Solid {
		let off = self.vertex_count() as u32;
		let positions: Vec<DVec3> = self.vertices.iter().chain(other.vertices.iter()).map(|v| v.position).collect();
		let mut faces: Vec<FaceLoops> = Vec::with_capacity(self.faces.len() + other.faces.len());
		for (s, base) in [(self, 0u32), (other, off)] {
			for f in s.faces() {
				let face = &s.faces[f.ix()];
				let loops: Vec<Vec<u32>> = std::iter::once(face.outer)
					.chain(face.inner.iter().copied())
					.map(|lp| s.loop_half_edges(lp).into_iter().map(|he| s.half_edges[he.ix()].origin.0 + base).collect())
					.collect();
				faces.push(FaceLoops { loops, surface: face.surface });
			}
		}
		Solid::from_faces_multiloop(positions, faces)
	}

	/// The supporting plane of a planar face as `(point, outward unit normal)`,
	/// derived from the face's boundary polygon (`point` is the polygon centroid,
	/// `normal` follows the winding, i.e. points outward for a well-formed solid).
	///
	/// This lets an assembly mate reference a face's *actual geometry* — pick a
	/// face, read its plane, and build a coincident/parallel constraint from it —
	/// rather than hand-computing the frame. Returns `None` for a degenerate face.
	pub fn face_plane(&self, f: FaceId) -> Option<(DVec3, DVec3)> {
		let poly = self.face_polygon(f);
		if poly.len() < 3 {
			return None;
		}
		// Newell's method: robust area-weighted normal that follows the winding.
		let mut n = DVec3::ZERO;
		for i in 0..poly.len() {
			let a = poly[i];
			let b = poly[(i + 1) % poly.len()];
			n.x += (a.y - b.y) * (a.z + b.z);
			n.y += (a.z - b.z) * (a.x + b.x);
			n.z += (a.x - b.x) * (a.y + b.y);
		}
		let n = n.normalize_or_zero();
		if n.length_squared() < 0.5 {
			return None;
		}
		let centroid = poly.iter().fold(DVec3::ZERO, |acc, &p| acc + p) / poly.len() as f64;
		Some((centroid, n))
	}

	/// The rotational axis of an axial face — cylinder, cone, or torus — as
	/// `(point_on_axis, unit_direction)`. Returns `None` for non-axial faces
	/// (planes, spheres). Companion to [`Self::face_plane`]: it lets an assembly
	/// build a concentric / axis-alignment mate from a face's *actual* analytic
	/// axis instead of a hand-supplied one.
	pub fn face_axis(&self, f: FaceId) -> Option<(DVec3, DVec3)> {
		let (point, axis) = match self.face(f).surface {
			Surface::Cylinder { origin, axis, .. } => (origin, axis),
			Surface::Cone { apex, axis, .. } => (apex, axis),
			Surface::Torus { center, axis, .. } => (center, axis),
			_ => return None,
		};
		let dir = axis.normalize_or_zero();
		(dir.length_squared() > 0.5).then_some((point, dir))
	}
}
