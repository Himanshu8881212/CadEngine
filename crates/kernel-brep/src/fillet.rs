// Copyright (c) LMCAD. Licensed under the MIT License.

//! Edge fillet — rounds a **named** convex edge of a solid with a constant-radius
//! cylindrical face.
//!
//! The operation is driven by an [`EdgeName`], not a transient [`crate::topo::EdgeId`]:
//! store the name, edit an upstream dimension, re-evaluate, and the same call
//! re-attaches the fillet to the corresponding edge of the edited part. That is
//! what makes topological naming *load-bearing* rather than a label — a feature can
//! say "round that edge" and have it stick across a rebuild.
//!
//! v1 handles the axis-aligned box case: a straight edge shared by two perpendicular
//! planar faces (every edge of a [`crate::build::cuboid`]). The fillet recedes the two
//! faces to their tangent lines, rounds the two end caps with a matching arc, and
//! inserts a quarter-cylinder between them — faceted into segments exactly as
//! [`crate::build::cylinder`] facets its side, so the result is a watertight,
//! consistently-oriented manifold with the fillet faces tagged [`Surface::Cylinder`].

use kernel_core::math::{DAffine3, DQuat, DVec3};

use crate::geom::Surface;
use crate::topo::{EdgeId, EdgeName, Face, FaceId, FaceInput, FaceLoops, FaceName, FaceSource, HalfEdgeId, Solid, VertexId};

/// Why a fillet could not be applied. A typed channel so an AI (or feature tree)
/// gets a precise, actionable reason instead of a panic or a silently-wrong solid.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FilletError {
	/// The edge name did not resolve to any edge in this solid.
	EdgeNotFound,
	/// The edge name resolved to more than one edge — disambiguate before filleting.
	EdgeAmbiguous,
	/// The radius is non-positive or not finite.
	BadRadius,
	/// The radius is too large to fit within an adjacent face.
	RadiusTooLarge,
	/// The edge is not a straight edge between two perpendicular planar faces — the
	/// only case v1 handles (e.g. a curved face, or a non-box dihedral).
	Unsupported,
}

/// Whether to round a named edge with a cylindrical arc or a single flat bevel.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum RoundKind {
	/// A constant-radius cylindrical fillet, faceted into segments.
	Fillet,
	/// A single planar bevel between the two tangent lines.
	Chamfer,
}

/// Round the edge named `edge` with radius `radius`, using the default facet count.
pub fn fillet_edge(solid: &Solid, edge: EdgeName, radius: f64) -> Result<Solid, FilletError> {
	fillet_edge_segments(solid, edge, radius, 16)
}

/// Round the edge named `edge` with radius `radius`, faceting the cylindrical fillet
/// into `segments` angular strips (clamped to at least 1).
pub fn fillet_edge_segments(solid: &Solid, edge: EdgeName, radius: f64, segments: usize) -> Result<Solid, FilletError> {
	round_edge(solid, edge, radius, segments, RoundKind::Fillet)
}

/// Chamfer (flat-bevel) the edge named `edge` with setback `radius` — the sibling of
/// [`fillet_edge`] using the same recede/cap machinery, but the cut face is a single
/// [`Surface::Plane`] bevel rather than a cylindrical arc.
pub fn chamfer_edge(solid: &Solid, edge: EdgeName, radius: f64) -> Result<Solid, FilletError> {
	round_edge(solid, edge, radius, 1, RoundKind::Chamfer)
}

/// Fillet the fragment of a split named edge nearest `witness`. When a boolean splits a
/// named edge into several collinear fragments (all bearing the same [`EdgeName`]),
/// [`fillet_edge`] returns [`FilletError::EdgeAmbiguous`]; this disambiguates by picking
/// the fragment whose closest point to `witness` is smallest and rounds just that one.
pub fn fillet_edge_near(solid: &Solid, edge: EdgeName, radius: f64, witness: DVec3) -> Result<Solid, FilletError> {
	round_edge_by_id(solid, nearest_named_edge(solid, edge, witness)?, radius, 16, RoundKind::Fillet)
}

/// Chamfer the fragment of a split named edge nearest `witness` (see [`fillet_edge_near`]).
pub fn chamfer_edge_near(solid: &Solid, edge: EdgeName, radius: f64, witness: DVec3) -> Result<Solid, FilletError> {
	round_edge_by_id(solid, nearest_named_edge(solid, edge, witness)?, radius, 1, RoundKind::Chamfer)
}

/// Resolve the persistent `edge` name to exactly one current edge, then round it. A name
/// that splits into several fragments is [`FilletError::EdgeAmbiguous`] — use the
/// `*_near` resolvers with a witness point to choose a fragment.
fn round_edge(solid: &Solid, edge: EdgeName, radius: f64, segments: usize, kind: RoundKind) -> Result<Solid, FilletError> {
	let matches = solid.edges_named(edge);
	let eid = match matches.len() {
		0 => return Err(FilletError::EdgeNotFound),
		1 => matches[0],
		_ => return Err(FilletError::EdgeAmbiguous),
	};
	round_edge_by_id(solid, eid, radius, segments, kind)
}

/// The edge bearing `edge` whose segment lies closest to `witness`, or
/// [`FilletError::EdgeNotFound`] when the name resolves to nothing.
fn nearest_named_edge(solid: &Solid, edge: EdgeName, witness: DVec3) -> Result<EdgeId, FilletError> {
	solid
		.edges_named(edge)
		.into_iter()
		.min_by(|&a, &b| {
			edge_point_distance(solid, a, witness)
				.partial_cmp(&edge_point_distance(solid, b, witness))
				.unwrap_or(std::cmp::Ordering::Equal)
		})
		.ok_or(FilletError::EdgeNotFound)
}

/// Number of distinct faces meeting at vertex `v` (its valence). A simple corner has
/// valence 3; a higher value means a feature junction the corner-rebuild can't fillet.
fn vertex_valence(solid: &Solid, v: VertexId) -> usize {
	(0..solid.half_edge_count() as u32)
		.filter(|&h| solid.half_edge(HalfEdgeId(h)).origin == v)
		.map(|h| solid.half_edge(HalfEdgeId(h)).face)
		.collect::<std::collections::BTreeSet<_>>()
		.len()
}

/// Distance from `p` to edge `eid`'s line segment.
fn edge_point_distance(solid: &Solid, eid: EdgeId, p: DVec3) -> f64 {
	let he = *solid.half_edge(solid.edge(eid).half_edge);
	let a = solid.position(he.origin);
	let b = solid.position(solid.half_edge(he.next).origin);
	let ab = b - a;
	let t = if ab.length_squared() > 1e-18 { ((p - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0) } else { 0.0 };
	(p - (a + ab * t)).length()
}

/// Shared geometry core: recede the two faces to their tangent lines, round the caps,
/// and bridge edge `eid` with `segments` strips (cylinder for a fillet, one bevel plane
/// for a chamfer). Operates on an already-resolved [`EdgeId`].
fn round_edge_by_id(solid: &Solid, eid: EdgeId, radius: f64, segments: usize, kind: RoundKind) -> Result<Solid, FilletError> {
	if !radius.is_finite() || radius <= 0.0 {
		return Err(FilletError::BadRadius);
	}
	// The rebuild below re-emits EVERY face through `Solid::from_faces`, which
	// carries one loop per face — so a face with inner (hole) loops anywhere in
	// the solid would have its holes silently dropped, and the result comes back
	// closed=false / non-manifold with a NEGATIVE material cut (measured on a
	// plain `extrude_with_holes` plate: 22 faces, 2 holed, cut −73 mm³). Refuse
	// loudly instead of handing back corrupt topology; a multi-loop-aware
	// rebuild is the fix, and until it lands this is an honest `Unsupported`.
	if solid.faces().any(|f| !solid.face(f).inner.is_empty()) {
		return Err(FilletError::Unsupported);
	}
	let segments = segments.max(1);

	// The edge's two faces, their plane normals, and its endpoints.
	let he1 = *solid.half_edge(solid.edge(eid).half_edge);
	let twin = he1.twin.ok_or(FilletError::Unsupported)?;
	let fa = he1.face;
	let fb = solid.half_edge(twin).face;
	let na = plane_normal(solid.face(fa))?;
	let nb = plane_normal(solid.face(fb))?;
	let vs = solid.position(he1.origin);
	let ve = solid.position(solid.half_edge(he1.next).origin);
	let along = ve - vs;
	let len = along.length();
	if len < 1e-12 {
		return Err(FilletError::Unsupported);
	}
	let axis = along / len;

	// 3. The edge must run along both faces' shared direction (na, nb ⟂ axis). The
	//    dihedral may be ANY angle, but a near-flat / reflex pocket (where 1 + na·nb → 0)
	//    has no fillet and is rejected.
	if na.dot(axis).abs() > 1e-6 || nb.dot(axis).abs() > 1e-6 {
		return Err(FilletError::Unsupported);
	}
	let denom = 1.0 + na.dot(nb);
	if denom < 1e-6 {
		return Err(FilletError::Unsupported);
	}

	// 3a. The edge must be CONVEX — a fillet removes material. Each face's outward normal
	//     must point away from the other face's interior; a concave/reflex edge (which a
	//     fillet would ADD material to) is a different operation, so reject it.
	let face_tangent = |f: FaceId| -> DVec3 {
		let poly = solid.face_polygon(f);
		let c: DVec3 = poly.iter().copied().sum::<DVec3>() / poly.len().max(1) as f64;
		let d = c - vs;
		(d - axis * d.dot(axis)).normalize_or_zero()
	};
	if na.dot(face_tangent(fb)) > -1e-9 || nb.dot(face_tangent(fa)) > -1e-9 {
		return Err(FilletError::Unsupported);
	}

	// 3b. The corner-replacement rebuild only handles TRIVALENT endpoints (a simple
	//     3-face corner: the two edge faces plus one cap). A higher-valence endpoint —
	//     e.g. where a boolean feature splits the edge — would have several faces
	//     meeting the vertex, and replacing the corner in all of them corrupts the
	//     topology. Reject cleanly rather than emit a broken solid; high-valence-endpoint
	//     filleting is a separate (local-edit) capability.
	let vs_id = he1.origin;
	let ve_id = solid.half_edge(he1.next).origin;
	if vertex_valence(solid, vs_id) != 3 || vertex_valence(solid, ve_id) != 3 {
		return Err(FilletError::Unsupported);
	}

	// 4. Fillet geometry for a general convex dihedral. The radius-r cylinder is tangent
	//    to both planes, so its axis sits at C = edge − r·(na+nb)/(1+na·nb) — the inward
	//    angle bisector, distance r/sin(θ/2) from the edge (θ = interior dihedral). The
	//    tangent lines are Ta = C + r·na on face A and Tb = C + r·nb on face B; the
	//    rounded profile is the arc Ta→Tb on the cylinder, swept through φ = ∠(na, nb) in
	//    the orthonormal basis (na, w) with w ⟂ na toward nb. (na·nb = 0 ⇒ the box case.)
	let cs = vs - (na + nb) * (radius / denom);
	let cphi = na.dot(nb).clamp(-1.0, 1.0);
	let w = (nb - na * cphi).normalize_or_zero();
	if w.length_squared() < 0.5 {
		return Err(FilletError::Unsupported); // na ∥ nb (no dihedral)
	}
	let phi = cphi.acos();
	let arc_s: Vec<DVec3> = (0..=segments)
		.map(|k| {
			let t = phi * k as f64 / segments as f64;
			cs + (na * t.cos() + w * t.sin()) * radius
		})
		.collect();

	// 4b. The setback to each tangent line must fit within that face (else the recede
	//     would run off the face edge).
	let fits = |f: FaceId, tangent: DVec3| -> bool {
		let dir = tangent - vs;
		let setback = dir.length();
		if setback < 1e-12 {
			return true;
		}
		let rdir = dir / setback;
		let ext = solid.face_polygon(f).iter().map(|p| (*p - vs).dot(rdir)).fold(0.0_f64, f64::max);
		setback < ext - 1e-9
	};
	if !fits(fa, arc_s[0]) || !fits(fb, arc_s[segments]) {
		return Err(FilletError::RadiusTooLarge);
	}

	let arc_e: Vec<DVec3> = arc_s.iter().map(|&p| p + along).collect();

	let mut positions: Vec<DVec3> = Vec::new();
	let mut faces: Vec<FaceInput> = Vec::new();
	let mut provenance: Vec<FaceName> = Vec::new();

	// 6. Rebuild every original face, replacing the filleted edge's corner(s):
	//    face A/B recede to the tangent line; an end cap has its corner replaced by
	//    the whole arc; all other faces are copied unchanged.
	for f in solid.faces() {
		let poly = solid.face_polygon(f);
		let name = solid
			.face_name(f)
			.unwrap_or(FaceName { operand: FaceSource::Primitive, source_face: f.0 });
		let n = poly.len();
		let mut boundary: Vec<u32> = Vec::with_capacity(n + segments);
		for i in 0..n {
			let p = poly[i];
			let on_s = (p - vs).length() < 1e-9;
			let on_e = (p - ve).length() < 1e-9;
			if !on_s && !on_e {
				boundary.push(intern(&mut positions, p));
				continue;
			}
			let arc = if on_s { &arc_s } else { &arc_e };
			if f == fa {
				boundary.push(intern(&mut positions, arc[0])); // tangent on face A
			} else if f == fb {
				boundary.push(intern(&mut positions, arc[segments])); // tangent on face B
			} else {
				// An end cap: replace the corner with the arc, ordered so its first
				// point continues from the previous boundary vertex (keeps it simple).
				let prev = poly[(i + n - 1) % n];
				let forward = (prev - arc[0]).length() <= (prev - arc[segments]).length();
				if forward {
					for &q in arc.iter() {
						boundary.push(intern(&mut positions, q));
					}
				} else {
					for &q in arc.iter().rev() {
						boundary.push(intern(&mut positions, q));
					}
				}
			}
		}
		faces.push(FaceInput { boundary, surface: solid.face(f).surface });
		provenance.push(name);
	}

	// 7. Bridge the two arc rings with strips: a faceted quarter-cylinder for a fillet,
	//    or one flat bevel plane for a chamfer (segments == 1). All strips share one
	//    logical name so `faces_named` re-selects the whole fillet/chamfer.
	let bridge_surface = match kind {
		RoundKind::Fillet => Surface::Cylinder { origin: cs, axis, radius },
		RoundKind::Chamfer => Surface::Plane { origin: arc_s[0], normal: (na + nb).normalize_or_zero() },
	};
	let fillet_name = FaceName { operand: FaceSource::Primitive, source_face: solid.face_count() as u32 };
	for k in 0..segments {
		let quad = [arc_s[k], arc_s[k + 1], arc_e[k + 1], arc_e[k]];
		let mid = phi * (k as f64 + 0.5) / segments as f64;
		let outward = na * mid.cos() + w * mid.sin();
		let boundary = oriented(&quad, outward).iter().map(|&p| intern(&mut positions, p)).collect();
		faces.push(FaceInput { boundary, surface: bridge_surface });
		provenance.push(fillet_name);
	}

	let mut result = Solid::from_faces(positions, faces);
	result.set_provenance(provenance);
	Ok(result)
}

/// One row of a torus rim-fillet band: the quarter-circle arc of `tube_seg + 1` points on the
/// tube circle of radius `minor` centred at `center + er * major`, sweeping `ψ = 0..π/2` from
/// the radial tangent (`radial_dir·er` — `+er` on a boss wall, `−er` on a bore wall, i.e. the
/// torus's outer or inner/saddle side) to the axial tangent (`+axis`, on the planar cap).
/// Every point lies exactly on `Surface::Torus { center, axis, major, minor }`. The shared
/// geometry core of [`rim_fillet_band`] (which samples uniform ring angles),
/// [`fillet_circular_rim`] and [`fillet_circular_rim_concave`] (which sample the detected
/// rim's own corner-vertex directions, so band ring vertices coincide exactly with the
/// wall's).
fn torus_band_row(center: DVec3, axis: DVec3, major: f64, minor: f64, er: DVec3, radial_dir: f64, tube_seg: usize) -> Vec<DVec3> {
	use std::f64::consts::FRAC_PI_2;
	let tube_center = center + er * major;
	(0..=tube_seg)
		.map(|j| {
			let psi = FRAC_PI_2 * j as f64 / tube_seg as f64;
			// Tube cross-section spans (radial_dir·er, axis): ψ=0 → the wall, ψ=π/2 → the cap.
			tube_center + (er * (radial_dir * psi.cos()) + axis * psi.sin()) * minor
		})
		.collect()
}

/// The surface geometry of a **rim fillet** — the rolling-ball fillet of a *circular* edge
/// (where a cylindrical wall of radius `major + minor` meets a planar cap), which is a quarter
/// of a torus rather than the straight cylinder of an edge fillet. Returns the analytic
/// [`Surface::Torus`] (centre on the axis, ring radius `major`, tube radius `minor` = the fillet
/// radius) and a `(ring_seg+1) × (tube_seg+1)` grid of band positions: the ring sweeps the full
/// `0..2π`, and the tube sweeps `ψ = 0..π/2` from the radial-outward tangent on the wall
/// (`radius = major + minor`) to the axial tangent on the cap (`radius = major`, offset `minor`
/// along the axis from `center`).
///
/// This is the geometry core of a curved-edge (rim) fillet. The straight-edge [`fillet_edge`]
/// handles planar–planar edges; for filleting a detected rim **in place** on an existing solid
/// (a boss on a plate, a bare cylinder) see [`fillet_circular_rim`], which consumes the same
/// band core ([`torus_band_row`]) phase-locked to the rim's actual vertices.
pub fn rim_fillet_band(center: DVec3, axis: DVec3, major: f64, minor: f64, ring_seg: usize, tube_seg: usize) -> (Surface, Vec<Vec<DVec3>>) {
	use std::f64::consts::TAU;
	let axis = axis.normalize_or_zero();
	// An orthonormal basis of the ring plane (perpendicular to the axis).
	let t = if axis.x.abs() < 0.9 { DVec3::X } else { DVec3::Y };
	let e1 = (t - axis * t.dot(axis)).normalize_or_zero();
	let e2 = axis.cross(e1);
	let ring_seg = ring_seg.max(1);
	let tube_seg = tube_seg.max(1);
	let grid = (0..=ring_seg)
		.map(|i| {
			let theta = TAU * i as f64 / ring_seg as f64;
			let er = e1 * theta.cos() + e2 * theta.sin(); // radial-outward in the ring plane
			torus_band_row(center, axis, major, minor, er, 1.0, tube_seg)
		})
		.collect();
	(Surface::Torus { center, axis, major, minor }, grid)
}

/// One detected rim-edge group of [`circular_rim_groups`]: every edge shared by a tagged
/// [`Surface::Cylinder`] wall and a [`Surface::Plane`] cap perpendicular to its axis,
/// accumulated per cap face (orientation — boss vs bore — is judged by the callers).
struct RimGroup {
	/// Cylinder origin, unit axis, radius shared by all the group's wall faces.
	cyl: (DVec3, DVec3, f64),
	walls: Vec<FaceId>,
	verts: Vec<VertexId>,
	edges_per_vertex: std::collections::BTreeMap<VertexId, usize>,
	/// The rim edges as vertex pairs — the chain a concave (multi-cap-piece) rim walks to
	/// prove it is one closed loop.
	chain: Vec<(VertexId, VertexId)>,
	/// All rim edges lie on the SAME cylinder (axes may differ in sign).
	consistent: bool,
}

/// A validated circular rim ready for the band rebuild — the shared output of
/// [`fillet_circular_rim`]'s and [`fillet_circular_rim_concave`]'s scope checks.
struct Rim {
	/// The planar face(s) carrying the rim on the cap side: exactly one whole-disc cap for a
	/// convex boss rim; possibly several pieces of one split annular cap around a bore.
	caps: Vec<FaceId>,
	walls: Vec<FaceId>,
	/// Rim vertices, CCW about `up`.
	ring: Vec<VertexId>,
	/// Cap-only vertices consumed by a grown bore hole (split-line vertices of cap pieces
	/// lying inside the grown annulus); the rebuild drops them from every cap loop. Both
	/// pieces sharing a split line drop the same vertices, so their shared boundary stays
	/// shared (straightened) and the cap union is unchanged. Validated and filled by the
	/// concave path; always empty for a convex boss rim.
	cap_consumed: Vec<VertexId>,
	/// Rim circle centre (axis ∩ cap plane).
	center: DVec3,
	/// Cap outward unit normal (= ±cylinder axis, exact).
	up: DVec3,
	/// Cylinder (rim circle) radius.
	radius: f64,
}

/// Detection step shared by [`fillet_circular_rim`] and [`fillet_circular_rim_concave`]:
/// group the solid's rim edges (a [`Surface::Cylinder`] wall meeting a [`Surface::Plane`]
/// cap whose winding normal is parallel to the wall axis) by cap face. Purely structural —
/// convex/concave orientation and scope limits are the callers' job.
fn circular_rim_groups(solid: &Solid) -> std::collections::BTreeMap<FaceId, RimGroup> {
	use std::collections::BTreeMap;
	let mut groups: BTreeMap<FaceId, RimGroup> = BTreeMap::new();
	for e in solid.edges() {
		let he = *solid.half_edge(solid.edge(e).half_edge);
		let Some(twin) = he.twin else { continue };
		let (fa, fb) = (he.face, solid.half_edge(twin).face);
		let pair = match (solid.face(fa).surface, solid.face(fb).surface) {
			(Surface::Cylinder { origin, axis, radius }, Surface::Plane { .. }) => Some((fb, fa, origin, axis, radius)),
			(Surface::Plane { .. }, Surface::Cylinder { origin, axis, radius }) => Some((fa, fb, origin, axis, radius)),
			_ => None,
		};
		let Some((cap, wall, o, a, r)) = pair else { continue };
		let a = a.normalize_or_zero();
		if a.length_squared() < 0.5 {
			continue;
		}
		// The cap must be perpendicular to the wall axis (its winding normal ∥ axis).
		if newell(&solid.face_polygon(cap)).dot(a).abs() < 1.0 - 1e-6 {
			continue;
		}
		let (v0, v1) = (he.origin, solid.half_edge(he.next).origin);
		let g = groups.entry(cap).or_insert_with(|| RimGroup {
			cyl: (o, a, r),
			walls: Vec::new(),
			verts: Vec::new(),
			edges_per_vertex: BTreeMap::new(),
			chain: Vec::new(),
			consistent: true,
		});
		// All rim edges of one cap must lie on the SAME cylinder (axes may differ in sign).
		let (go, ga, gr) = g.cyl;
		let same_axis = ga.cross(a).length() < 1e-9;
		let on_axis = (o - go).cross(ga).length() < 1e-6;
		if !(same_axis && on_axis && (gr - r).abs() < 1e-9) {
			g.consistent = false;
		}
		if !g.walls.contains(&wall) {
			g.walls.push(wall);
		}
		g.chain.push((v0, v1));
		for v in [v0, v1] {
			if !g.verts.contains(&v) {
				g.verts.push(v);
			}
			*g.edges_per_vertex.entry(v).or_insert(0) += 1;
		}
	}
	groups
}

/// Rim vertices ordered CCW about `up` around `center` (any fixed in-plane basis works).
fn order_ring_ccw(solid: &Solid, verts: &[VertexId], center: DVec3, up: DVec3) -> Vec<VertexId> {
	let t = if up.x.abs() < 0.9 { DVec3::X } else { DVec3::Y };
	let e1 = (t - up * t.dot(up)).normalize_or_zero();
	let e2 = up.cross(e1);
	let mut ring = verts.to_vec();
	ring.sort_by(|&va, &vb| {
		let ang = |v: VertexId| {
			let rel = solid.position(v) - center;
			rel.dot(e2).atan2(rel.dot(e1))
		};
		ang(va).partial_cmp(&ang(vb)).unwrap_or(std::cmp::Ordering::Equal)
	});
	ring
}

/// One step of a rebuilt cap-piece boundary: an untouched original vertex, or a rim-ring
/// CORNER (by ring index) on the moved hole boundary.
enum CapStep {
	Vert(VertexId),
	Ring(usize),
}

/// The rim-ring frame shared by [`clip_cap_loop`]'s callers: the ring's vertex→index map,
/// the consumed split-line vertices, the per-vertex corner alias, the corner subsequence,
/// the unit radial per ring vertex, and the moved hole circle (centre, axis, radius).
struct RingFrame<'a> {
	rim_index: &'a std::collections::BTreeMap<VertexId, usize>,
	consumed: &'a [VertexId],
	alias: &'a [usize],
	corners: &'a [usize],
	ers: &'a [DVec3],
	center: DVec3,
	up: DVec3,
	grown_radius: f64,
}

/// Rebuild one cap-piece loop around the MOVED hole boundary — the shared sectioning core
/// of [`fillet_circular_rim_concave`]'s fold guard and [`rebuild_with_rim_band`]'s cap
/// emission, so the simulation and the rebuild cannot disagree.
///
/// Vertices classify as rim (in `rim_index`), consumed (cap-only split-line vertices inside
/// the grown annulus) or KEPT. An all-rim loop (a whole-disc convex cap, a washer's inner
/// hole loop) maps each vertex to its alias corner wholesale. Otherwise every maximal
/// cyclic run of non-kept vertices — a split-line descent, the piece's rim arc, and the
/// ascent out — is replaced by a walk along grown CORNERS from the run's entry crossing to
/// its exit crossing (each crossing = where the boundary crosses the grown circle, snapped
/// to the nearest corner), in the rim arc's own direction. Two pieces sharing a split line
/// compute the same crossing from the same segment, so their replacement boundaries stay
/// shared, and the walks of adjacent pieces meet at the shared snapped corner — the grown
/// hole polygon is tiled exactly once (the caller cross-checks that globally). `None` if a
/// section carries no rim vertex (a notch that never touches the ring — direction would be
/// a guess), a mixed rim/consumed loop has no kept vertex (the piece vanishes into the
/// hole), or the result degenerates below a triangle.
fn clip_cap_loop(solid: &Solid, loop_verts: &[VertexId], frame: &RingFrame) -> Option<Vec<CapStep>> {
	let RingFrame { rim_index, consumed, alias, corners, ers, center, up, grown_radius } = *frame;
	let n = loop_verts.len();
	if n < 3 {
		return None;
	}
	let kept: Vec<bool> = loop_verts
		.iter()
		.map(|v| !rim_index.contains_key(v) && !consumed.contains(v))
		.collect();
	let mut steps: Vec<CapStep> = Vec::new();
	if kept.iter().all(|&k| !k) {
		// Wholesale: every vertex must be rim (a rim+consumed-only loop has no surviving
		// boundary — the piece would vanish into the hole).
		for v in loop_verts {
			let &k = rim_index.get(v)?;
			steps.push(CapStep::Ring(alias[k]));
		}
	} else {
		let in_plane = |p: DVec3| {
			let rel = p - center;
			rel - up * rel.dot(up)
		};
		// Unit radial direction of the crossing of segment `outside → inside` with the grown
		// circle (the outside end is a kept vertex, the inside end a section vertex).
		let crossing_dir = |outside: DVec3, inside: DVec3| -> DVec3 {
			let (pa, pb) = (in_plane(outside), in_plane(inside));
			let d = pb - pa;
			let (ab, bb) = (pa.dot(d), d.dot(d));
			let c0 = pa.dot(pa) - grown_radius * grown_radius;
			if c0 <= 0.0 || bb < 1e-18 {
				return pa.normalize_or_zero(); // outside end already on/inside the circle
			}
			let disc = ab * ab - bb * c0;
			if disc <= 0.0 {
				return pa.normalize_or_zero();
			}
			let sd = disc.sqrt();
			let r1 = (-ab - sd) / bb;
			let s = if (0.0..=1.0).contains(&r1) { r1 } else { ((-ab + sd) / bb).clamp(0.0, 1.0) };
			(pa + d * s).normalize_or_zero()
		};
		// Corner-list position with the largest radial alignment to `u`.
		let nearest_corner = |u: DVec3| -> usize {
			(0..corners.len())
				.max_by(|&i, &j| u.dot(ers[corners[i]]).partial_cmp(&u.dot(ers[corners[j]])).unwrap_or(std::cmp::Ordering::Equal))
				.unwrap_or(0)
		};
		let walk = |from: usize, to: usize, d: i64| -> Vec<usize> {
			let len = corners.len() as i64;
			let mut out = Vec::new();
			let mut c = from as i64;
			loop {
				out.push(corners[c as usize]);
				if c as usize == to || out.len() > corners.len() {
					break;
				}
				c = (c + d).rem_euclid(len);
			}
			out
		};
		// Rotate to start at a kept vertex so sections never wrap the seam.
		let start = (0..n).find(|&i| kept[i])?;
		let at = |i: usize| (start + i) % n;
		let mut i = 0;
		while i < n {
			if kept[at(i)] {
				steps.push(CapStep::Vert(loop_verts[at(i)]));
				i += 1;
				continue;
			}
			let sec_start = i;
			while i < n && !kept[at(i)] {
				i += 1;
			}
			let sec_end = i; // exclusive; at(sec_end) (or the loop start) is kept
			let rim_ring: Vec<usize> = (sec_start..sec_end).filter_map(|k| rim_index.get(&loop_verts[at(k)]).copied()).collect();
			if rim_ring.is_empty() {
				return None; // a notch that never touches the ring — direction would be a guess
			}
			let prev_kept = solid.position(loop_verts[at(sec_start + n - 1)]);
			let next_kept = solid.position(loop_verts[at(sec_end % n)]);
			let u_in = crossing_dir(prev_kept, solid.position(loop_verts[at(sec_start)]));
			let u_out = crossing_dir(next_kept, solid.position(loop_verts[at(sec_end - 1)]));
			let (ce_in, ce_out) = (nearest_corner(u_in), nearest_corner(u_out));
			let nring = ers.len() as i64;
			let d = if rim_ring.len() >= 2 {
				// Consecutive section rim vertices are chain-adjacent: their (small) ring step
				// gives the arc's rotational direction.
				let step = (rim_ring[1] as i64 - rim_ring[0] as i64).rem_euclid(nring);
				if step <= nring / 2 {
					1
				} else {
					-1
				}
			} else {
				// A single rim vertex: take the direction whose corner walk passes its own
				// alias corner.
				let c_star = alias[rim_ring[0]];
				if walk(ce_in, ce_out, 1).contains(&c_star) {
					1
				} else {
					-1
				}
			};
			for c in walk(ce_in, ce_out, d) {
				steps.push(CapStep::Ring(c));
			}
		}
	}
	// Deduplicate consecutive equal steps (aliased ring vertices collapse onto one corner)
	// and the closure repeat.
	let same = |x: &CapStep, y: &CapStep| match (x, y) {
		(CapStep::Vert(a), CapStep::Vert(b)) => a == b,
		(CapStep::Ring(a), CapStep::Ring(b)) => a == b,
		_ => false,
	};
	let mut dedup: Vec<CapStep> = Vec::with_capacity(steps.len());
	for s in steps {
		if dedup.last().is_none_or(|l| !same(l, &s)) {
			dedup.push(s);
		}
	}
	while dedup.len() > 2 && same(&dedup[0], dedup.last().unwrap()) {
		dedup.pop();
	}
	if dedup.len() < 3 {
		return None; // the piece degenerated below a triangle
	}
	Some(dedup)
}

/// The qualifying rim nearest `witness` (distance to the rim circle itself), shared by the
/// convex and concave rim fillets.
fn pick_rim_near(rims: Vec<Rim>, witness: DVec3) -> Option<Rim> {
	rims.into_iter().min_by(|x, y| {
		let d = |rim: &Rim| {
			let rel = witness - rim.center;
			let axial = rel.dot(rim.up);
			((rel - rim.up * axial).length() - rim.radius).hypot(axial)
		};
		d(x).partial_cmp(&d(y)).unwrap_or(std::cmp::Ordering::Equal)
	})
}

/// Shared rebuild core of [`fillet_circular_rim`] (`radial_dir = 1.0`, a convex boss rim)
/// and [`fillet_circular_rim_concave`] (`radial_dir = -1.0`, a bore rim): build the exact
/// quarter-torus band phase-locked to the rim's own vertices and re-emit every face with the
/// rim ring remapped onto the band's tangent rings — wall faces to the ψ=0 ring (axial
/// setback `radius`), the cap to the ψ=π/2 ring (radial setback `−radial_dir·radius`, i.e.
/// the cap shrinks around a boss and its hole grows around a bore). The band tube circle has
/// ring radius `rim.radius − radial_dir·radius` and sits `radius` below the cap.
///
/// Ring vertices ON the rim circle (within the detection tolerance) are **corners** and
/// anchor rows generated by [`torus_band_row`] — every such row vertex lies exactly on the
/// tagged [`Surface::Torus`]. A boolean-built rim usually also carries extra non-corner
/// vertices ON the chords between corners: collinear ring subdivisions and the feet of the
/// split lines that partition an annular cap (which can land mid-chord). Each such vertex is
/// **aliased to its nearest corner's row**: since it lies exactly on the chord, dropping it
/// from the wall/cap boundaries changes no geometry (the chord is the same straight edge
/// with or without it), and a split-line foot merely slides along the rim — an in-plane
/// re-partition of the same cap region, volume-neutral. Aliasing (rather than lerping
/// interior rows for them) keeps every band vertex exactly on the torus — so the
/// divergence-theorem `torus_bulge` patches in [`exact_volume`] stay exact — and removes
/// collinear boundary vertices that per-face ear clipping would keep or skip inconsistently
/// (exact predicates on ~1e-16 collinearity noise), which would leak open mesh edges.
/// `None` if a face that is neither wall nor cap touches the rim, a rim vertex sits on the
/// axis, a non-corner vertex is not on its corners' chord, or fewer than three corners
/// exist.
fn rebuild_with_rim_band(solid: &Solid, rim: &Rim, radius: f64, radial_dir: f64, tube_seg: usize) -> Option<Solid> {
	use std::collections::BTreeMap;
	use std::f64::consts::FRAC_PI_2;
	let (up, rim_center) = (rim.up, rim.center);
	let n = rim.ring.len();
	let major = rim.radius - radial_dir * radius;
	let band_center = rim_center - up * radius;
	let band_surface = Surface::Torus { center: band_center, axis: up, major, minor: radius };
	let on_circle = 1e-6; // matches the detection tolerance
	let rel_in_plane: Vec<DVec3> = rim
		.ring
		.iter()
		.map(|&v| {
			let rel = solid.position(v) - rim_center;
			rel - up * rel.dot(up)
		})
		.collect();
	let ers: Vec<DVec3> = rel_in_plane.iter().map(|r| r.normalize_or_zero()).collect();
	if ers.iter().any(|er| er.length_squared() < 0.5) {
		return None;
	}
	let is_corner: Vec<bool> = rel_in_plane.iter().map(|r| (r.length() - rim.radius).abs() <= on_circle).collect();
	let corners: Vec<usize> = (0..n).filter(|&k| is_corner[k]).collect();
	if corners.len() < 3 {
		return None; // no inscribed polygon to anchor the band
	}
	let mut arcs: Vec<Vec<DVec3>> = vec![Vec::new(); n];
	for &k in &corners {
		arcs[k] = torus_band_row(band_center, up, major, radius, ers[k], radial_dir, tube_seg);
	}
	// Alias every non-corner ring vertex (a collinear ring subdivision or a mid-chord
	// split-line foot) to its NEAREST bracketing corner's row — geometry-preserving because
	// the vertex must lie exactly on the corners' chord (scope-checked here).
	let mut alias: Vec<usize> = (0..n).collect();
	for k in 0..n {
		if is_corner[k] {
			continue;
		}
		let prev = (1..n).map(|d| (k + n - d) % n).find(|&i| is_corner[i])?;
		let next = (1..n).map(|d| (k + d) % n).find(|&i| is_corner[i])?;
		if prev == next {
			return None; // a single corner cannot bracket a chord
		}
		let (a, b, p) = (solid.position(rim.ring[prev]), solid.position(rim.ring[next]), solid.position(rim.ring[k]));
		let ab = b - a;
		if ab.length_squared() < 1e-18 || (p - a).cross(ab).length() / ab.length() > on_circle {
			return None; // the vertex must sit ON its corners' chord
		}
		alias[k] = if (p - a).length_squared() <= (p - b).length_squared() { prev } else { next };
	}

	// Copy every face; remap the wall rim vertices to the ψ=0 tangent ring and rebuild the
	// cap loops around the moved hole via the shared sectioning core.
	let rim_index: BTreeMap<VertexId, usize> = rim.ring.iter().copied().enumerate().map(|(k, v)| (v, k)).collect();
	let mut positions: Vec<DVec3> = Vec::new();
	let mut faces: Vec<FaceLoops> = Vec::new();
	let mut provenance: Vec<FaceName> = Vec::new();
	for f in solid.faces() {
		let face = solid.face(f);
		let mut loops: Vec<Vec<u32>> = Vec::with_capacity(1 + face.inner.len());
		for lid in std::iter::once(face.outer).chain(face.inner.iter().copied()) {
			let mut boundary: Vec<u32> = Vec::new();
			let mut push = |p: DVec3, boundary: &mut Vec<u32>| {
				let idx = intern(&mut positions, p);
				// Aliased ring vertices collapse onto their corner; drop consecutive
				// duplicates so no degenerate (v→v) half-edge is emitted.
				if boundary.last() != Some(&idx) {
					boundary.push(idx);
				}
			};
			if rim.caps.contains(&f) {
				let frame = RingFrame {
					rim_index: &rim_index,
					consumed: &rim.cap_consumed,
					alias: &alias,
					corners: &corners,
					ers: &ers,
					center: rim.center,
					up,
					grown_radius: major,
				};
				let vs: Vec<VertexId> = solid.loop_half_edges(lid).iter().map(|&h| solid.half_edge(h).origin).collect();
				for step in clip_cap_loop(solid, &vs, &frame)? {
					let p = match step {
						CapStep::Vert(v) => solid.position(v),
						CapStep::Ring(c) => arcs[c][tube_seg], // cap tangent ring
					};
					push(p, &mut boundary);
				}
			} else {
				for he in solid.loop_half_edges(lid) {
					let v = solid.half_edge(he).origin;
					let p = match rim_index.get(&v) {
						Some(&k) if rim.walls.contains(&f) => arcs[alias[k]][0], // wall tangent ring
						Some(_) => return None, // a non-wall/cap face on the rim — out of scope
						None => solid.position(v),
					};
					push(p, &mut boundary);
				}
			}
			if boundary.len() > 2 && boundary.first() == boundary.last() {
				boundary.pop();
			}
			if boundary.len() < 3 {
				return None; // a cap sliver collapsed entirely — out of scope
			}
			loops.push(boundary);
		}
		faces.push(FaceLoops { loops, surface: face.surface });
		provenance.push(solid.face_name(f).unwrap_or(FaceName { operand: FaceSource::Primitive, source_face: f.0 }));
	}

	// Insert the band quads between consecutive CORNER arcs (aliased ring vertices have no
	// row of their own); one shared name for the fillet.
	let band_name = FaceName { operand: FaceSource::Primitive, source_face: solid.face_count() as u32 };
	for c in 0..corners.len() {
		let k = corners[c];
		let k1 = corners[(c + 1) % corners.len()];
		let er_mid = (ers[k] + ers[k1]).normalize_or_zero();
		for j in 0..tube_seg {
			let psi_mid = FRAC_PI_2 * (j as f64 + 0.5) / tube_seg as f64;
			let outward = er_mid * (radial_dir * psi_mid.cos()) + up * psi_mid.sin();
			let quad = [arcs[k][j], arcs[k1][j], arcs[k1][j + 1], arcs[k][j + 1]];
			let boundary = oriented(&quad, outward).iter().map(|&p| intern(&mut positions, p)).collect();
			faces.push(FaceLoops { loops: vec![boundary], surface: band_surface });
			provenance.push(band_name);
		}
	}
	let mut result = Solid::from_faces_multiloop(positions, faces);
	result.set_provenance(provenance);
	Some(result)
}

/// Fillet (round) a **circular convex rim** of `solid` — the closed ring of edges where a
/// cylindrical wall meets the planar cap at its end — with the exact rolling-ball **torus**
/// of radius `radius`, on solids that need NOT be bare primitives: a boss fused on a plate,
/// a pin on a bracket, any multi-feature body that carries the rim in its B-rep. This wires
/// the [`rim_fillet_band`] machinery into the fillet API:
///
/// 1. **Detect** the rim from the B-rep itself: every edge whose two faces are a
///    [`Surface::Cylinder`] wall and a [`Surface::Plane`] cap perpendicular to its axis,
///    grouped per cap face. `witness` picks the nearest qualifying rim circle when several
///    qualify (a bare cylinder has two).
/// 2. **Rebuild locally**: the wall facets are shortened by `radius` along the axis, the cap
///    shrunk by `radius` radially, and the quarter-torus band — every vertex generated by the
///    same [`torus_band_row`] core as [`rim_fillet_band`], so it lies *exactly* on the tagged
///    [`Surface::Torus`] — is inserted between the two tangent rings. The band is
///    phase-locked to the rim's own vertices (one arc per rim vertex), so the wall, band and
///    cap share ring vertices and the result is **watertight by construction**. Every other
///    face (inner hole loops included) is copied untouched, and face provenance is carried so
///    stored names keep resolving.
///
/// `arc_segments` facets the quarter arc; the ring keeps the wall's own vertex count/phase.
///
/// HONEST SCOPE — returns `None` for anything outside it:
/// - **Convex boss rims only**: the wall's outward normals must point radially outward
///   (material inside the cylinder) and the cap must sit at the end of the wall, facing away
///   from it. The concave junction where a boss meets its base plate is a different operation
///   (a fillet there *adds* material) and remains future work; rounding a bore's exit lip
///   (wall normal pointing inward) is [`fillet_circular_rim_concave`].
/// - The cap's outer boundary must be exactly the rim ring (a full circular cap) with no
///   inner loops, and every rim vertex must be trivalent (the cap plus two wall facets) —
///   rims interrupted by other features are not yet rebuilt.
/// - `radius` must fit: strictly less than the cap radius, and the wall must extend at least
///   `radius` below the cap.
pub fn fillet_circular_rim(solid: &Solid, witness: DVec3, radius: f64, arc_segments: usize) -> Option<Solid> {
	if !radius.is_finite() || radius <= 0.0 {
		return None;
	}
	let tube_seg = arc_segments.max(1);
	let tol = 1e-6;

	// --- 1. Detection: group rim edges (Cylinder wall ∧ Plane cap ⟂ axis) by cap face.
	let groups = circular_rim_groups(solid);

	// --- 2. Validate each candidate against the supported scope; keep qualifying rims.
	let mut rims: Vec<Rim> = Vec::new();
	'groups: for (cap, g) in &groups {
		let (o, a, r) = g.cyl;
		if !g.consistent || g.verts.len() < 3 || radius >= r - 1e-9 {
			continue;
		}
		let cap_face = solid.face(*cap);
		if !cap_face.inner.is_empty() {
			continue; // cap with holes (an annulus) — out of scope
		}
		let cap_poly = solid.face_polygon(*cap);
		// Cap outward direction, snapped onto the exact cylinder axis.
		let up = if newell(&cap_poly).dot(a) >= 0.0 { a } else { -a };
		// Rim circle centre: the cylinder axis pierced through the cap plane.
		let center = o + a * (cap_poly[0] - o).dot(a);
		// The cap's outer boundary must BE the rim ring: every vertex on the rim circle…
		for p in &cap_poly {
			let rel = *p - center;
			if rel.dot(up).abs() > tol || ((rel - up * rel.dot(up)).length() - r).abs() > tol {
				continue 'groups;
			}
		}
		// …each rim vertex trivalent and chained by exactly two rim edges…
		if !g.verts.iter().all(|&v| vertex_valence(solid, v) == 3 && g.edges_per_vertex.get(&v) == Some(&2)) {
			continue;
		}
		// …and the rim a closed ring covering the cap (same vertex sets).
		if g.verts.len() != cap_poly.len() {
			continue;
		}
		// Convex boss only: walls radially outward, ending AT the cap, at least `radius` deep.
		for &w in &g.walls {
			let wall_poly = solid.face_polygon(w);
			let centroid = wall_poly.iter().copied().sum::<DVec3>() / wall_poly.len() as f64;
			let rel = centroid - o;
			let radial = (rel - a * rel.dot(a)).normalize_or_zero();
			if newell(&wall_poly).dot(radial) <= 0.0 {
				continue 'groups; // a bore wall (outward normal points inward) — out of scope
			}
			for p in &wall_poly {
				let depth = (center - *p).dot(up); // 0 at the rim, > 0 below the cap
				if depth < -tol {
					continue 'groups; // wall continues past the cap — not an end rim (concave)
				}
				if depth > tol && depth < radius + 1e-9 {
					continue 'groups; // wall shorter than the fillet radius — does not fit
				}
			}
		}
		let ring = order_ring_ccw(solid, &g.verts, center, up);
		rims.push(Rim { caps: vec![*cap], walls: g.walls.clone(), ring, cap_consumed: Vec::new(), center, up, radius: r });
	}

	// --- 3. Pick the qualifying rim nearest the witness (distance to the rim circle), then
	//     rebuild around the exact torus band, phase-locked to the rim vertices: one quarter
	//     arc per rim vertex, from its wall tangent (ψ=0, axial setback `radius`) to its cap
	//     tangent (ψ=π/2, radial setback `radius`). Same core as `rim_fillet_band`.
	let rim = pick_rim_near(rims, witness)?;
	rebuild_with_rim_band(solid, &rim, radius, 1.0, tube_seg)
}

/// Fillet (round over) a **concave circular rim** of `solid` — the inner ring of edges where
/// a cylindrical BORE wall (a concave wall: outward normals point radially inward) pierces a
/// planar cap, e.g. the bore↔top-face lip of `hex_nut` — with the exact rolling-ball
/// **torus** of radius `radius`. The bore companion of [`fillet_circular_rim`]: the ball
/// rolls around the hole's lip tangent to both faces, so the band is the **saddle (inner)
/// quarter** of a torus with ring radius `bore_radius + radius`, centred `radius` below the
/// cap on the bore axis; the cap's hole loop grows by `radius`, the bore wall shortens by
/// `radius`, and the band — every vertex generated by the same [`torus_band_row`] core,
/// phase-locked to the rim's own corner vertices — bridges the two tangent rings,
/// watertight by construction, with face provenance carried so stored names keep resolving.
///
/// HONEST GEOMETRY NOTE: rounding a lip whose material wedge is 90° **removes** the corner
/// ring (cross-section `r²(1−π/4)` hugging the old lip, exact ring volume
/// `2π·[R·r²(1−π/4) + r³(5/6−π/4)]` by Pappus, R = bore radius) — the solid sheds it and the
/// bore (void) gains it. The concave junction that *adds* material (a boss meeting its base
/// plate, where the band would be tangent to the plate top and the boss wall) is a different
/// rebuild (the plate face keeps the boss circle as an inner loop) and remains future work.
///
/// Detection mirrors [`fillet_circular_rim`] with the orientations inverted, generalised to
/// the cap structure booleans actually emit: cutting a bore through a face usually SPLITS
/// the annular cap into several simply-connected planar pieces (no inner loop survives),
/// with extra on-chord ring vertices (collinear subdivisions and split-line feet) where the
/// split lines land on the bore ring, and the split polylines fanning outward THROUGH the
/// annulus the fillet consumes — so rim groups are merged per (cylinder, cap plane) across
/// cap pieces, the rebuild ALIASES the on-chord ring vertices to their nearest corners, and
/// each piece's hole-touching boundary section — split-line descent, rim arc, ascent — is
/// replaced by a walk along grown corners between its circle crossings (cap-only split-line
/// vertices inside the grown annulus are consumed; see [`rebuild_with_rim_band`] and
/// [`clip_cap_loop`]); both pieces sharing a split line compute the same crossing, so the
/// re-partition stays watertight and the cap union is unchanged (volume-neutral). `witness`
/// picks the nearest qualifying rim (a through-bore has two, one per cap). HONEST SCOPE —
/// `None` for anything outside it:
/// - the wall's outward normals must point radially **inward** (a bore; boss rims belong to
///   [`fillet_circular_rim`]) and every wall must end AT the cap and extend at least
///   `radius` past it (a shallower bore cannot absorb the axial setback);
/// - the rim must be ONE closed chain (every rim vertex chained by exactly two rim edges,
///   one connected loop) with at least three vertices exactly on the bore circle and the
///   rest exactly on their corners' chords — lips interrupted by other features (a keyway
///   through the bore) are not rebuilt;
/// - the grown hole must have room: after simulating the rebuilt cap boundaries, every
///   surviving cap edge must stay outside the grown hole polygon (its chords reach in to
///   `(bore+radius)·cos(Δθ_max/2)`, Δθ_max = the widest corner gap), no two cap-plane
///   edges may properly cross, and the pieces' corner walks must tile the grown polygon
///   exactly once — a hexagon flat or another hole inside the grown radius, a foreign
///   (non-cap) feature in the consumed zone, or split polylines that would fold over each
///   other all reject;
/// - faces other than the bore walls and the cap pieces touching the rim ⇒ `None`.
pub fn fillet_circular_rim_concave(solid: &Solid, witness: DVec3, radius: f64, arc_segments: usize) -> Option<Solid> {
	use std::collections::{BTreeMap, BTreeSet};
	if !radius.is_finite() || radius <= 0.0 {
		return None;
	}
	let tube_seg = arc_segments.max(1);
	let tol = 1e-6;

	// --- 1. Detection: the same structural grouping as the convex rim fillet, then MERGE
	//     the per-cap-face groups that share one cylinder and one cap plane (the split
	//     pieces of a bored face all carry arcs of the same rim).
	let groups = circular_rim_groups(solid);
	struct Merged {
		/// Rim circle centre (axis ∩ cap plane) — the merge key together with the cylinder.
		center: DVec3,
		cyl: (DVec3, DVec3, f64),
		caps: Vec<FaceId>,
		walls: Vec<FaceId>,
		verts: Vec<VertexId>,
		edges_per_vertex: BTreeMap<VertexId, usize>,
		chain: Vec<(VertexId, VertexId)>,
		consistent: bool,
	}
	let mut merged: Vec<Merged> = Vec::new();
	for (cap, g) in &groups {
		let (o, a, r) = g.cyl;
		let center = o + a * (solid.face_polygon(*cap)[0] - o).dot(a);
		match merged.iter_mut().find(|m| {
			let (_, ma, mr) = m.cyl;
			ma.cross(a).length() < 1e-9 && (mr - r).abs() < 1e-9 && (m.center - center).length() < tol
		}) {
			Some(m) => {
				m.consistent &= g.consistent;
				m.caps.push(*cap);
				for w in &g.walls {
					if !m.walls.contains(w) {
						m.walls.push(*w);
					}
				}
				for v in &g.verts {
					if !m.verts.contains(v) {
						m.verts.push(*v);
					}
				}
				for (v, c) in &g.edges_per_vertex {
					*m.edges_per_vertex.entry(*v).or_insert(0) += *c;
				}
				m.chain.extend_from_slice(&g.chain);
			}
			None => merged.push(Merged {
				center,
				cyl: (o, a, r),
				caps: vec![*cap],
				walls: g.walls.clone(),
				verts: g.verts.clone(),
				edges_per_vertex: g.edges_per_vertex.clone(),
				chain: g.chain.clone(),
				consistent: g.consistent,
			}),
		}
	}

	// --- 2. Validate each merged candidate against the CONCAVE scope.
	let mut rims: Vec<Rim> = Vec::new();
	'merged: for m in &merged {
		let (o, a, r) = m.cyl;
		if !m.consistent || m.verts.len() < 3 {
			continue;
		}
		let center = m.center;
		// Cap outward direction, snapped onto the exact cylinder axis; all cap pieces of one
		// planar cap must agree on it.
		let up = if newell(&solid.face_polygon(m.caps[0])).dot(a) >= 0.0 { a } else { -a };
		if m.caps.iter().any(|&c| newell(&solid.face_polygon(c)).dot(up) < 1.0 - 1e-6) {
			continue;
		}
		// One closed chain: every rim vertex chained by exactly two rim edges…
		if !m.verts.iter().all(|&v| m.edges_per_vertex.get(&v) == Some(&2)) {
			continue;
		}
		// …and the rim edges form a single loop covering every rim vertex.
		{
			let mut adj: BTreeMap<VertexId, Vec<VertexId>> = BTreeMap::new();
			for &(x, y) in &m.chain {
				adj.entry(x).or_default().push(y);
				adj.entry(y).or_default().push(x);
			}
			let Some(&start) = adj.keys().next() else { continue };
			let (mut prev, mut cur, mut count) = (start, adj[&start][0], 1usize);
			while cur != start && count <= m.verts.len() {
				let nbrs = &adj[&cur];
				let next = if nbrs[0] == prev { nbrs[1] } else { nbrs[0] };
				prev = cur;
				cur = next;
				count += 1;
			}
			if cur != start || count != m.verts.len() {
				continue; // several loops or an open chain — not one whole bore rim
			}
		}
		// Every rim vertex in the cap plane and not outside the bore circle; count the
		// corners (exactly on the circle — the rebuild validates the rest sit on chords).
		let mut corner_dirs: Vec<DVec3> = Vec::new();
		for &v in &m.verts {
			let rel = solid.position(v) - center;
			let in_plane = rel - up * rel.dot(up);
			if rel.dot(up).abs() > tol || in_plane.length() > r + tol {
				continue 'merged;
			}
			if (in_plane.length() - r).abs() <= tol {
				corner_dirs.push(in_plane.normalize_or_zero());
			}
		}
		if corner_dirs.len() < 3 {
			continue;
		}
		// Concave bore only: wall normals radially inward, ending AT the cap, ≥ `radius` deep.
		for &w in &m.walls {
			let wall_poly = solid.face_polygon(w);
			let centroid = wall_poly.iter().copied().sum::<DVec3>() / wall_poly.len() as f64;
			let rel = centroid - o;
			let radial = (rel - a * rel.dot(a)).normalize_or_zero();
			if newell(&wall_poly).dot(radial) >= 0.0 {
				continue 'merged; // a boss wall (outward) — fillet_circular_rim's case
			}
			for p in &wall_poly {
				let depth = (center - *p).dot(up); // 0 at the rim, > 0 below the cap
				if depth < -tol {
					continue 'merged; // wall continues past the cap — not an exit lip
				}
				if depth > tol && depth < radius + 1e-9 {
					continue 'merged; // bore shallower than the fillet radius — does not fit
				}
			}
		}
		// The grown hole CONSUMES the annulus [bore, bore+radius] of the cap. Cap-only
		// split-line vertices inside it (the boolean's cap pieces fan their boundaries out
		// from the bore ring, with vertices mid-annulus) are REMOVED from the cap loops,
		// and each piece's whole hole-touching boundary section is replaced by a walk along
		// grown corners between its crossing points (see [`clip_cap_loop`]) — both pieces
		// sharing a split line compute the same crossing, so their shared boundary stays
		// shared and the cap union is unchanged: an in-plane re-partition, volume-neutral.
		let (e1, e2) = {
			let t = if up.x.abs() < 0.9 { DVec3::X } else { DVec3::Y };
			let e1 = (t - up * t.dot(up)).normalize_or_zero();
			(e1, up.cross(e1))
		};
		let mut angles: Vec<f64> = corner_dirs.iter().map(|d| d.dot(e2).atan2(d.dot(e1))).collect();
		angles.sort_by(|x, y| x.partial_cmp(y).unwrap_or(std::cmp::Ordering::Equal));
		let max_gap = angles
			.iter()
			.zip(angles.iter().cycle().skip(1))
			.take(angles.len())
			.map(|(x, y)| (y - x).rem_euclid(std::f64::consts::TAU))
			.fold(0.0_f64, f64::max);
		let clear_radius = (r + radius) * (0.5 * max_gap).cos();
		let rim_set: BTreeSet<VertexId> = m.verts.iter().copied().collect();
		// Ring + per-vertex corner alias, mirroring the rebuild: a non-corner ring vertex (a
		// collinear subdivision or a split-line foot ON the chord) is aliased to its nearest
		// bracketing corner, so the guard below simulates the same boundaries the rebuild
		// emits (via the shared [`clip_cap_loop`]).
		let ring = order_ring_ccw(solid, &m.verts, center, up);
		let nring = ring.len();
		let ers: Vec<DVec3> = ring
			.iter()
			.map(|&v| {
				let rel = solid.position(v) - center;
				(rel - up * rel.dot(up)).normalize_or_zero()
			})
			.collect();
		let ring_corner: Vec<bool> = ring
			.iter()
			.map(|&v| {
				let rel = solid.position(v) - center;
				((rel - up * rel.dot(up)).length() - r).abs() <= tol
			})
			.collect();
		let corners: Vec<usize> = (0..nring).filter(|&k| ring_corner[k]).collect();
		let mut alias: Vec<usize> = (0..nring).collect();
		for k in 0..nring {
			if ring_corner[k] {
				continue;
			}
			let prev = (1..nring).map(|d| (k + nring - d) % nring).find(|&i| ring_corner[i]);
			let next = (1..nring).map(|d| (k + d) % nring).find(|&i| ring_corner[i]);
			let (Some(prev), Some(next)) = (prev, next) else { continue 'merged };
			let p = solid.position(ring[k]);
			alias[k] = if (p - solid.position(ring[prev])).length_squared() <= (p - solid.position(ring[next])).length_squared() {
				prev
			} else {
				next
			};
		}
		let rim_ring_index: BTreeMap<VertexId, usize> = ring.iter().copied().enumerate().map(|(k, v)| (v, k)).collect();
		// Collect the consumed vertices: cap-loop vertices inside the grown annulus that are
		// neither rim vertices nor reaching past it. Each must be CAP-ONLY (only faces of
		// this cap carry it) — a third feature meeting the consumed zone is out of scope.
		let mut consumed: Vec<VertexId> = Vec::new();
		for &c in &m.caps {
			let cf = solid.face(c);
			for lid in std::iter::once(cf.outer).chain(cf.inner.iter().copied()) {
				for he in solid.loop_half_edges(lid) {
					let v = solid.half_edge(he).origin;
					if rim_set.contains(&v) || consumed.contains(&v) {
						continue;
					}
					let rel = solid.position(v) - center;
					let in_plane = rel - up * rel.dot(up);
					if in_plane.length() >= r + radius - tol {
						continue; // already clear of the grown hole
					}
					if rel.dot(up).abs() > tol || in_plane.length() < r - tol {
						continue 'merged; // off-plane or inside the bore — malformed cap
					}
					consumed.push(v);
				}
			}
		}
		// A consumed vertex must be cap-only: every face of the solid that uses it is a piece
		// of this cap (walls touch the cap plane only along rim edges, so anything else is a
		// foreign feature in the consumed zone).
		if !consumed.is_empty() {
			for i in 0..solid.half_edge_count() as u32 {
				let he = solid.half_edge(HalfEdgeId(i));
				if consumed.contains(&he.origin) && !m.caps.contains(&he.face) {
					continue 'merged;
				}
			}
		}
		// Simulate the rebuild's cap boundaries via the shared [`clip_cap_loop`] and reject
		// any rim whose result would misbehave:
		// (a) every surviving split/outer edge must stay outside the grown hole polygon;
		// (b) no two cap-plane edges may properly cross (split polylines that weave
		//     tangentially through the consumed annulus would fold when replaced — honest
		//     rejection instead of a self-overlapping cap);
		// (c) the pieces' corner walks must tile the grown hole polygon exactly once, so
		//     every band edge pairs with exactly one cap edge.
		// A comparable endpoint key for the fold guard: original vertex or ring corner.
		type StepKey = (u8, u64);
		let mut segs: Vec<(StepKey, StepKey, DVec3, DVec3)> = Vec::new();
		let mut walked: BTreeSet<(usize, usize)> = BTreeSet::new();
		let frame = RingFrame {
			rim_index: &rim_ring_index,
			consumed: &consumed,
			alias: &alias,
			corners: &corners,
			ers: &ers,
			center,
			up,
			grown_radius: r + radius,
		};
		for &c in &m.caps {
			let cf = solid.face(c);
			for lid in std::iter::once(cf.outer).chain(cf.inner.iter().copied()) {
				let vs: Vec<VertexId> = solid.loop_half_edges(lid).iter().map(|&h| solid.half_edge(h).origin).collect();
				let Some(steps) = clip_cap_loop(solid, &vs, &frame) else {
					continue 'merged; // a piece vanished or a notch never touched the ring
				};
				let key = |s: &CapStep| match s {
					CapStep::Vert(v) => (0u8, v.0 as u64),
					CapStep::Ring(c) => (1u8, *c as u64),
				};
				let pos = |s: &CapStep| match s {
					CapStep::Vert(v) => solid.position(*v),
					CapStep::Ring(c) => center + ers[*c] * (r + radius),
				};
				for i in 0..steps.len() {
					let (sa, sb) = (&steps[i], &steps[(i + 1) % steps.len()]);
					if let (CapStep::Ring(ca), CapStep::Ring(cb)) = (sa, sb) {
						// A hole-boundary chord: band-paired by construction; (c) each ring-
						// adjacent corner pair must be walked exactly once across all pieces.
						let pair = (*ca.min(cb), *ca.max(cb));
						if !walked.insert(pair) {
							continue 'merged; // walked twice — the tiling overlaps
						}
						continue;
					}
					let (pa, pb) = (pos(sa), pos(sb));
					let da = {
						let rel = pa - center;
						rel - up * rel.dot(up)
					};
					let db = {
						let rel = pb - center;
						rel - up * rel.dot(up)
					};
					let dab = db - da;
					let t = if dab.length_squared() > 1e-18 { (-da.dot(dab) / dab.length_squared()).clamp(0.0, 1.0) } else { 0.0 };
					if (da + dab * t).length() < clear_radius - tol {
						continue 'merged; // (a) the grown hole would swallow this edge
					}
					segs.push((key(sa), key(sb), pa, pb));
				}
			}
		}
		// (c) the walks must cover every ring-adjacent corner pair (with the duplicate check
		// above, equal cardinality means an exact tiling).
		if walked.len() != corners.len() {
			continue 'merged;
		}
		let cross2 = |p: DVec3, q: DVec3| p.dot(e1) * q.dot(e2) - p.dot(e2) * q.dot(e1);
		let mut folds = false;
		'cross: for i in 0..segs.len() {
			for j in (i + 1)..segs.len() {
				let (a0, a1, pa0, pa1) = segs[i];
				let (b0, b1, pb0, pb1) = segs[j];
				if a0 == b0 || a0 == b1 || a1 == b0 || a1 == b1 {
					continue; // sharing an endpoint is adjacency, not a crossing
				}
				let (d1, d2) = (cross2(pa1 - pa0, pb0 - pa0), cross2(pa1 - pa0, pb1 - pa0));
				let (d3, d4) = (cross2(pb1 - pb0, pa0 - pb0), cross2(pb1 - pb0, pa1 - pb0));
				if d1 * d2 < -1e-18 && d3 * d4 < -1e-18 {
					folds = true;
					break 'cross;
				}
			}
		}
		if folds {
			continue 'merged; // (b) replaced split lines would fold across each other
		}
		rims.push(Rim { caps: m.caps.clone(), walls: m.walls.clone(), ring, cap_consumed: consumed, center, up, radius: r });
	}

	// --- 3. Nearest qualifying rim, then the shared band rebuild on the torus's saddle side
	//     (radial_dir = −1): wall tangents at ψ=0 (`−e_r`, axial setback `radius`), cap
	//     tangents at ψ=π/2 (the hole grown to `bore + radius`).
	let rim = pick_rim_near(rims, witness)?;
	rebuild_with_rim_band(solid, &rim, radius, -1.0, tube_seg)
}

/// Detect a **circular rim edge** of `solid` — where a cylindrical face meets a planar cap — and
/// return `(center, axis, radius)` of that rim: the rim circle's centre (the cylinder axis
/// projected onto the cap plane), the cylinder axis, and the cylinder radius. `toward` selects
/// which cap (e.g. `+Z` for the top); the cap is the planar face perpendicular to the axis
/// furthest along `toward`. `None` if the solid has no analytic cylindrical face or no
/// perpendicular cap.
///
/// Feeds [`rim_fillet_band`] (Step 2 of a curved-edge fillet — detection only; it does not modify
/// the solid). The centre is the axis∩cap intersection (correct even when the cap face's own
/// centroid is elsewhere). NOTE: this needs an analytic [`Surface::Cylinder`] face — it finds the
/// rim of a **primitive** cylinder; a bore cut by a boolean is planarized (its wall loses the
/// cylinder tag) and is not detected until the boolean pipeline preserves analytic surfaces.
pub fn cylinder_rim(solid: &Solid, toward: DVec3) -> Option<(DVec3, DVec3, f64)> {
	let toward = toward.normalize_or_zero();
	let (origin, axis, radius) = solid.faces().find_map(|f| match solid.face(f).surface {
		Surface::Cylinder { origin, axis, radius } => Some((origin, axis.normalize_or_zero(), radius)),
		_ => None,
	})?;
	// The cap = a planar face perpendicular to the axis, furthest along `toward`.
	let mut best: Option<(f64, DVec3)> = None;
	for f in solid.faces() {
		if let Surface::Plane { origin: po, normal } = solid.face(f).surface {
			if normal.normalize_or_zero().dot(axis).abs() > 0.999 {
				let score = po.dot(toward);
				if best.is_none_or(|(s, _)| score > s) {
					best = Some((score, po));
				}
			}
		}
	}
	let cap_point = best?.1;
	// The rim centre is where the cylinder axis pierces the cap plane (axis ⟂ cap).
	let center = origin + axis * (cap_point - origin).dot(axis);
	Some((center, axis, radius))
}

/// Round the **top rim** (the cap toward `+axis`) of an existing **primitive cylinder** `solid` by
/// a curved-edge fillet of radius `fillet` — the in-place counterpart of [`crate::filleted_cylinder`].
/// The cylinder's axis, radius and end caps are detected via [`cylinder_rim`]; the rounded solid is
/// rebuilt as a surface of revolution and transformed back onto the cylinder's pose, so the result
/// is watertight and genus-0 by construction (no fragile face surgery). `None` if `solid` is not a
/// recognisable primitive cylinder (e.g. a boolean-planarized bore — see [`cylinder_rim`]).
pub fn fillet_cylinder_rim(solid: &Solid, fillet: f64, segments: usize, arc_segments: usize) -> Option<Solid> {
	// Probe once to learn the axis + radius (toward is irrelevant to those); then resolve the two
	// caps with toward = ±axis so the top/bottom centres are picked stably.
	let (_, axis, radius) = cylinder_rim(solid, DVec3::Z)?;
	if axis.length_squared() < 0.5 {
		return None;
	}
	let (top, _, _) = cylinder_rim(solid, axis)?;
	let (bottom, _, _) = cylinder_rim(solid, -axis)?;
	let height = (top - bottom).dot(axis).abs();
	if height < 1e-9 {
		return None;
	}
	// Build a +Z filleted cylinder (base at the origin), then rotate +Z → axis and place its base
	// at the cylinder's bottom-cap centre. The fillet sits on the local top → the +axis rim.
	let local = crate::filleted_cylinder(radius, height, fillet, segments, arc_segments);
	let xform = DAffine3::from_rotation_translation(DQuat::from_rotation_arc(DVec3::Z, axis), bottom);
	Some(local.transformed(xform))
}

/// Chamfer (45°-bevel) the **top rim** of an existing **primitive cylinder** `solid` by `chamfer` —
/// the cut-edge counterpart of [`fillet_cylinder_rim`]. The cylinder is detected via [`cylinder_rim`]
/// and rebuilt as a surface of revolution ([`crate::chamfered_cylinder`]) transformed onto its pose,
/// so the result is watertight and genus-0 by construction. `None` if `solid` is not a recognisable
/// primitive cylinder.
pub fn chamfer_cylinder_rim(solid: &Solid, chamfer: f64, segments: usize) -> Option<Solid> {
	let (_, axis, radius) = cylinder_rim(solid, DVec3::Z)?;
	if axis.length_squared() < 0.5 {
		return None;
	}
	let (top, _, _) = cylinder_rim(solid, axis)?;
	let (bottom, _, _) = cylinder_rim(solid, -axis)?;
	let height = (top - bottom).dot(axis).abs();
	if height < 1e-9 {
		return None;
	}
	let local = crate::chamfered_cylinder(radius, height, chamfer, segments);
	let xform = DAffine3::from_rotation_translation(DQuat::from_rotation_arc(DVec3::Z, axis), bottom);
	Some(local.transformed(xform))
}

/// The outward plane normal of a planar face, or [`FilletError::Unsupported`].
fn plane_normal(face: &Face) -> Result<DVec3, FilletError> {
	match face.surface {
		Surface::Plane { normal, .. } => Ok(normal),
		_ => Err(FilletError::Unsupported),
	}
}

/// Intern a position, returning the index of an existing coincident vertex (within a
/// weld tolerance) or pushing a new one — so faces that meet share vertex indices and
/// the half-edge twin matcher pairs them.
fn intern(positions: &mut Vec<DVec3>, p: DVec3) -> u32 {
	for (i, &q) in positions.iter().enumerate() {
		if (p - q).length() < 1e-9 {
			return i as u32;
		}
	}
	positions.push(p);
	(positions.len() - 1) as u32
}

/// Newell's area-weighted polygon normal (CCW ⇒ right-hand normal).
fn newell(poly: &[DVec3]) -> DVec3 {
	let mut n = DVec3::ZERO;
	let len = poly.len();
	for i in 0..len {
		let c = poly[i];
		let d = poly[(i + 1) % len];
		n.x += (c.y - d.y) * (c.z + d.z);
		n.y += (c.z - d.z) * (c.x + d.x);
		n.z += (c.x - d.x) * (c.y + d.y);
	}
	n.normalize_or_zero()
}

/// Return the quad wound so its polygon normal agrees with `outward`.
fn oriented(quad: &[DVec3; 4], outward: DVec3) -> Vec<DVec3> {
	if newell(quad).dot(outward) < 0.0 {
		quad.iter().rev().copied().collect()
	} else {
		quad.to_vec()
	}
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::build::{cuboid, cylinder};
	use crate::tessellate::tessellate_default;
	use crate::topo::EdgeName;
	use crate::validate::{exact_volume, validate, volume};

	/// A NON-primitive multi-feature fixture: a square plate (half-extent `a`, thickness `t`)
	/// with a cylindrical boss (radius `r`, height `h`, `seg` facets) fused on top, built
	/// directly as one watertight B-rep — the plate's top face carries the boss-base circle
	/// as an inner hole loop, the boss wall is tagged [`Surface::Cylinder`]. This is the
	/// "boss on a plate" a curved-rim fillet must handle beyond bare primitives.
	fn plate_with_boss(a: f64, t: f64, r: f64, h: f64, seg: usize) -> Solid {
		use std::f64::consts::TAU;
		let mut pos: Vec<DVec3> = vec![
			DVec3::new(-a, -a, 0.0),
			DVec3::new(a, -a, 0.0),
			DVec3::new(a, a, 0.0),
			DVec3::new(-a, a, 0.0),
			DVec3::new(-a, -a, t),
			DVec3::new(a, -a, t),
			DVec3::new(a, a, t),
			DVec3::new(-a, a, t),
		];
		let base = pos.len() as u32; // boss-base ring on the plate top (z = t)
		for k in 0..seg {
			let th = TAU * k as f64 / seg as f64;
			pos.push(DVec3::new(r * th.cos(), r * th.sin(), t));
		}
		let top = pos.len() as u32; // boss-top rim ring (z = t + h)
		for k in 0..seg {
			let th = TAU * k as f64 / seg as f64;
			pos.push(DVec3::new(r * th.cos(), r * th.sin(), t + h));
		}
		let s = seg as u32;
		let mut faces: Vec<FaceLoops> = vec![
			FaceLoops { loops: vec![vec![0, 3, 2, 1]], surface: Surface::Plane { origin: DVec3::ZERO, normal: -DVec3::Z } },
			// Plate top: square outer loop + the boss-base circle as a (reversed) hole loop.
			FaceLoops {
				loops: vec![vec![4, 5, 6, 7], (0..s).rev().map(|k| base + k).collect()],
				surface: Surface::Plane { origin: DVec3::new(0.0, 0.0, t), normal: DVec3::Z },
			},
			FaceLoops { loops: vec![vec![0, 1, 5, 4]], surface: Surface::Plane { origin: DVec3::new(0.0, -a, 0.0), normal: -DVec3::Y } },
			FaceLoops { loops: vec![vec![1, 2, 6, 5]], surface: Surface::Plane { origin: DVec3::new(a, 0.0, 0.0), normal: DVec3::X } },
			FaceLoops { loops: vec![vec![2, 3, 7, 6]], surface: Surface::Plane { origin: DVec3::new(0.0, a, 0.0), normal: DVec3::Y } },
			FaceLoops { loops: vec![vec![3, 0, 4, 7]], surface: Surface::Plane { origin: DVec3::new(-a, 0.0, 0.0), normal: -DVec3::X } },
		];
		let cyl = Surface::Cylinder { origin: DVec3::new(0.0, 0.0, t), axis: DVec3::Z, radius: r };
		for k in 0..s {
			let k1 = (k + 1) % s;
			faces.push(FaceLoops { loops: vec![vec![base + k, base + k1, top + k1, top + k]], surface: cyl });
		}
		faces.push(FaceLoops {
			loops: vec![(0..s).map(|k| top + k).collect()],
			surface: Surface::Plane { origin: DVec3::new(0.0, 0.0, t + h), normal: DVec3::Z },
		});
		Solid::from_faces_multiloop(pos, faces)
	}

	#[test]
	fn fillet_circular_rim_rounds_a_boss_on_a_plate() {
		use std::f64::consts::{PI, TAU};
		// THE curved-rim generalisation: an exact torus fillet on a rim of a NON-primitive
		// multi-feature solid (a boss fused on a plate). The rim is detected from the B-rep
		// (cylinder wall ∧ plane cap); the concave boss-base junction must NOT qualify. The
		// result must be valid, watertight, torus-tagged with every band vertex EXACTLY on
		// the tagged torus, leave all non-rim faces untouched, and remove exactly the
		// rolling-ball corner ring: sharp − filleted = 2π·[r²(R−r/2) − (πr²/4)(R−r) − r³/3]
		// (Pappus over the square-corner-minus-quarter-disc cross-section).
		//
		// NOTE on the volume oracle: the plate-top face carries the boss circle as an INNER
		// loop, exact_volume's documented R4 frontier (it over-counts multi-loop faces
		// today). The fillet never touches that face, so the sharp−filleted exact_volume
		// DIFFERENCE cancels the R4 term entirely and isolates the fillet's removal — which
		// is then MACHINE-exact against the Pappus closed form (≤1e-9): torus_bulge's patch
		// flux plus its ψ-row lateral slivers closes exactly against the wall's cylinder
		// lenses and the cap plane. The faceted tessellation separately enforces the strict
		// sharp-vs-bound envelope.
		let (a, t, rb, hb, seg) = (10.0, 3.0, 4.0, 6.0, 48);
		let part = plate_with_boss(a, t, rb, hb, seg);
		let v0 = validate(&part);
		let sharp_true = (2.0 * a) * (2.0 * a) * t + PI * rb * rb * hb;
		assert!(
			v0.is_valid() && v0.genus == 0 && tessellate_default(&part).is_watertight() && (volume(&part) - sharp_true).abs() / sharp_true < 0.01,
			"fixture: {v0:?} tessellated volume {} (closed form {sharp_true})",
			volume(&part)
		);

		let fr = 1.5;
		let rounded = fillet_circular_rim(&part, DVec3::new(0.0, 0.0, t + hb), fr, 8).expect("the boss top rim fillets");
		let v = validate(&rounded);
		let torus_faces: Vec<_> = rounded.faces().filter(|&f| matches!(rounded.face(f).surface, Surface::Torus { .. })).collect();
		let surf = rounded.face(torus_faces[0]).surface;
		let Surface::Torus { center, axis, major, minor } = surf else { unreachable!() };
		let max_off = torus_faces
			.iter()
			.flat_map(|&f| rounded.face_polygon(f))
			.fold(0.0_f64, |m, p| m.max(surf.unsigned_distance(p)));
		let removed_exact = exact_volume(&part) - exact_volume(&rounded);
		let removed_tess = volume(&part) - volume(&rounded);
		let removed_true = TAU * (fr * fr * (rb - 0.5 * fr) - (PI * fr * fr / 4.0) * (rb - fr) - fr.powi(3) / 3.0);
		let ring_bound = TAU * rb * fr * fr; // the whole square corner ring — a generous cap
		assert!(
			v.is_valid()
				&& v.genus == 0
				&& tessellate_default(&rounded).is_watertight()
				&& torus_faces.len() == seg * 8
				&& rounded.face_count() == part.face_count() + seg * 8
				&& (center - DVec3::new(0.0, 0.0, t + hb - fr)).length() < 1e-9
				&& axis.cross(DVec3::Z).length() < 1e-9
				&& (major - (rb - fr)).abs() < 1e-9
				&& (minor - fr).abs() < 1e-9
				&& max_off < 1e-9
				&& removed_tess > 0.0
				&& removed_tess < ring_bound
				&& (removed_exact - removed_true).abs() / removed_true < 1e-9,
			"boss rim fillet: {v:?} wt={} torus_faces={} faces {}→{} center={center:?} axis={axis:?} major={major} minor={minor} \
			 band_off={max_off:.2e} removed_exact={removed_exact} (closed form {removed_true}) removed_tess={removed_tess} (bound {ring_bound})",
			tessellate_default(&rounded).is_watertight(),
			torus_faces.len(),
			part.face_count(),
			rounded.face_count()
		);
	}

	#[test]
	fn fillet_circular_rim_works_on_a_union_built_boss() {
		use crate::booleans::union;
		// The same capability on GENUINE boolean output (not a hand-built fixture): union a
		// cylinder onto a plate — the boolean keeps the wall's Surface::Cylinder tags — and
		// fillet the boss's top rim in place. Valid, watertight, torus-tagged, and the
		// removed material sits strictly inside the corner-ring bound. This is the chain a
		// parametric feature tree actually produces (plate ∪ boss → rim fillet).
		let plate = cuboid(DVec3::new(-10.0, -10.0, 0.0), DVec3::new(10.0, 10.0, 3.0));
		let boss = cylinder(DVec3::new(0.0, 0.0, 3.0), DVec3::Z, 4.0, 6.0, 48);
		let u = union(&plate, &boss);
		let walls = u.faces().filter(|&f| matches!(u.face(f).surface, Surface::Cylinder { .. })).count();
		let fr = 1.5;
		let rounded = fillet_circular_rim(&u, DVec3::new(0.0, 0.0, 9.0), fr, 8).expect("a union-built boss rim fillets");
		let v = validate(&rounded);
		let torus_faces = rounded.faces().filter(|&f| matches!(rounded.face(f).surface, Surface::Torus { .. })).count();
		let removed = volume(&u) - volume(&rounded);
		let ring_bound = std::f64::consts::TAU * 4.0 * fr * fr;
		assert!(
			walls == 48
				&& v.is_valid()
				&& v.genus == 0
				&& tessellate_default(&rounded).is_watertight()
				&& torus_faces == 48 * 8
				&& removed > 0.0
				&& removed < ring_bound,
			"union-boss rim fillet: walls={walls} {v:?} wt={} torus_faces={torus_faces} removed={removed} (bound {ring_bound})",
			tessellate_default(&rounded).is_watertight()
		);
	}

	#[test]
	fn fillet_circular_rim_matches_the_revolve_built_fillet_on_a_bare_cylinder() {
		// Cross-validation against the trusted path: on a bare primitive cylinder the local
		// rim surgery must produce the SAME solid (volume-identical, valid, watertight) as
		// fillet_cylinder_rim's full revolve rebuild — both sample the identical torus rings.
		// A bare cylinder has TWO qualifying convex rims; the witness picks top vs bottom,
		// and the bottom fillet's torus sits at base+r with its band curving down (−Z cap).
		let (base, r, h, seg) = (DVec3::new(3.0, -2.0, 1.0), 5.0, 12.0, 48);
		let cyl = cylinder(base, DVec3::Z, r, h, seg);
		let fr = 2.0;
		let surgery = fillet_circular_rim(&cyl, base + DVec3::Z * (h + 1.0), fr, 8).expect("top rim fillets in place");
		let revolve = fillet_cylinder_rim(&cyl, fr, seg, 8).expect("the revolve rebuild fillets");
		let (vs, vr) = (volume(&surgery), volume(&revolve));
		let v = validate(&surgery);
		let bottom = fillet_circular_rim(&cyl, base - DVec3::Z, fr, 8).expect("bottom rim fillets in place");
		let vb = validate(&bottom);
		let bottom_torus = bottom
			.faces()
			.find_map(|f| match bottom.face(f).surface {
				Surface::Torus { center, axis, .. } => Some((center, axis)),
				_ => None,
			})
			.expect("bottom fillet carries a torus band");
		assert!(
			v.is_valid()
				&& v.genus == 0
				&& tessellate_default(&surgery).is_watertight()
				&& (vs - vr).abs() < 1e-9 * vr.abs()
				&& vb.is_valid()
				&& tessellate_default(&bottom).is_watertight()
				&& (bottom_torus.0 - (base + DVec3::Z * fr)).length() < 1e-9
				&& bottom_torus.1.dot(DVec3::Z) < -0.999
				&& volume(&bottom) < volume(&cyl),
			"surgery {v:?} vol={vs} vs revolve vol={vr}; bottom {vb:?} torus at {:?} axis {:?}",
			bottom_torus.0,
			bottom_torus.1
		);
	}

	#[test]
	fn fillet_circular_rim_honestly_rejects_out_of_scope_rims() {
		use crate::booleans::difference;
		// The documented scope limits return None instead of a wrong solid:
		// (1) no cylindrical wall at all; (2) a fillet radius the cap cannot absorb (≥ R);
		// (3) a wall shorter than the fillet radius; (4) a BORE's exit lip — the wall's
		// outward normal points inward (and the surrounding cap carries the hole as an
		// inner loop), which is rounding-a-hole, a different (future) operation.
		let box_only = cuboid(DVec3::splat(-2.0), DVec3::splat(2.0));
		let tall = plate_with_boss(10.0, 3.0, 4.0, 6.0, 32);
		let stubby = plate_with_boss(10.0, 3.0, 4.0, 2.0, 32);
		let bored = difference(
			&cuboid(DVec3::new(-5.0, -5.0, 0.0), DVec3::new(5.0, 5.0, 3.0)),
			&cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, 2.0, 5.0, 32),
		);
		assert!(
			fillet_circular_rim(&box_only, DVec3::ZERO, 1.0, 8).is_none()
				&& fillet_circular_rim(&tall, DVec3::new(0.0, 0.0, 9.0), 4.0, 8).is_none()
				&& fillet_circular_rim(&stubby, DVec3::new(0.0, 0.0, 5.0), 3.0, 8).is_none()
				&& fillet_circular_rim(&bored, DVec3::new(2.0, 0.0, 3.0), 0.5, 8).is_none(),
			"out-of-scope rims must be rejected, not mangled"
		);
	}

	/// A hex-nut fixture mirroring kernel-model's `parts::hex_nut(width, height, bore)`
	/// (kernel-model depends on this crate, so the part is rebuilt here): a hexagonal prism
	/// of across-flats `width` (apothem `width/2`), bored through by a Ø`bore` 48-segment
	/// cylinder — GENUINE boolean output: each cap is split into several planar pieces
	/// around the bore (no inner loop survives) and the bore ring carries mid-chord
	/// T-junction vertices where the split lines land, exactly the structure a concave rim
	/// fillet must cope with.
	fn hex_nut_fixture(width: f64, height: f64, bore: f64) -> Solid {
		use crate::booleans::difference;
		use crate::build::extrude;
		use kernel_core::math::DVec2;
		use std::f64::consts::PI;
		let circumradius = (width * 0.5) / (PI / 6.0).cos();
		let hexagon: Vec<DVec2> = (0..6)
			.map(|i| {
				let a = PI / 6.0 + i as f64 * PI / 3.0;
				DVec2::new(circumradius * a.cos(), circumradius * a.sin())
			})
			.collect();
		let body = extrude(&hexagon, height);
		let hole = cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, bore * 0.5, height + 2.0, 48);
		difference(&body, &hole)
	}

	#[test]
	fn fillet_circular_rim_concave_rounds_a_bore_exit_lip() {
		use std::f64::consts::{PI, TAU};
		// THE concave-rim case: round over the lip where a BORE pierces a planar cap —
		// parts::hex_nut(16, 8, 10)'s bore↔top-face rim (rebuilt as hex_nut_fixture). The
		// ball rolls around the hole tangent to wall and cap: the cap's hole loop grows by r,
		// the bore wall shortens by r, and the saddle quarter of the torus {center (0,0,h−r),
		// axis Z, major bore_r + r, minor r} bridges them. Rounding this 90°-wedge lip SHEDS
		// the corner ring to the bore: cross-section r²(1−π/4) hugging the old lip, exact
		// Pappus volume V_ring = 2π·[R·r²(1−π/4) + r³(5/6−π/4)] (∫(R+u) dA over the
		// square-minus-quarter-disc section; the u-moment integrates to r³(5/6−π/4)).
		//
		// The boolean-built rim carries 52 extra on-chord ring vertices (collinear ring
		// subdivisions plus 4 split-line feet near corners) and split polylines fanning
		// through the consumed annulus — the rebuild ALIASES the former to corners and
		// replaces the latter with corner walks (see rebuild_with_rim_band/clip_cap_loop),
		// so the band is 48 strips of 8 with EVERY vertex exactly on the tagged torus.
		// Assertions: valid genus-1 watertight; band torus-tagged with the derived
		// parameters; every band point within 1e-9 of the analytic torus; the exact_volume
		// DROP machine-exact against the Pappus ring (≤1e-9 — torus_bulge's patch flux plus
		// its ψ-row lateral slivers closes exactly against the wall's cylinder lenses and
		// the cap plane); the faceted tessellation inside the strict sharp-vs-bound
		// envelope; and the BOTTOM lip fillets symmetrically (witness below → torus r above
		// the bottom cap, band curving down).
		let (w, h, bore) = (16.0, 8.0, 10.0);
		let nut = hex_nut_fixture(w, h, bore);
		let v0 = validate(&nut);
		assert!(
			v0.is_valid() && v0.genus == 1 && tessellate_default(&nut).is_watertight(),
			"fixture must be a watertight genus-1 nut: {v0:?}"
		);

		let (rb, fr) = (bore * 0.5, 1.0);
		let rounded = fillet_circular_rim_concave(&nut, DVec3::new(0.0, 0.0, h + 1.0), fr, 8).expect("the bore-top lip fillets");
		let v = validate(&rounded);
		let torus_faces: Vec<_> = rounded.faces().filter(|&f| matches!(rounded.face(f).surface, Surface::Torus { .. })).collect();
		let surf = rounded.face(torus_faces[0]).surface;
		let Surface::Torus { center, axis, major, minor } = surf else { unreachable!() };
		let max_off = torus_faces
			.iter()
			.flat_map(|&f| rounded.face_polygon(f))
			.fold(0.0_f64, |m, p| m.max(surf.unsigned_distance(p)));
		let removed_exact = exact_volume(&nut).abs() - exact_volume(&rounded).abs();
		let removed_tess = volume(&nut).abs() - volume(&rounded).abs();
		let ring_true = TAU * (rb * fr * fr * (1.0 - PI / 4.0) + fr.powi(3) * (5.0 / 6.0 - PI / 4.0));
		let ring_bound = TAU * (rb + fr) * fr * fr; // the whole square corner ring at the grown radius
		let bottom = fillet_circular_rim_concave(&nut, DVec3::new(0.0, 0.0, -1.0), fr, 8).expect("the bore-bottom lip fillets");
		let vb = validate(&bottom);
		let bottom_torus = bottom
			.faces()
			.find_map(|f| match bottom.face(f).surface {
				Surface::Torus { center, axis, .. } => Some((center, axis)),
				_ => None,
			})
			.expect("bottom fillet carries a torus band");
		assert!(
			v.is_valid()
				&& v.genus == 1
				&& tessellate_default(&rounded).is_watertight()
				&& torus_faces.len() == 48 * 8
				&& rounded.face_count() == nut.face_count() + 48 * 8
				&& (center - DVec3::new(0.0, 0.0, h - fr)).length() < 1e-9
				&& axis.cross(DVec3::Z).length() < 1e-9
				&& (major - (rb + fr)).abs() < 1e-9
				&& (minor - fr).abs() < 1e-9
				&& max_off < 1e-9
				&& removed_tess > 0.0
				&& removed_tess < ring_bound
				&& (removed_exact - ring_true).abs() / ring_true < 1e-9
				&& vb.is_valid()
				&& tessellate_default(&bottom).is_watertight()
				&& (bottom_torus.0 - DVec3::new(0.0, 0.0, fr)).length() < 1e-9
				&& bottom_torus.1.dot(DVec3::Z) < -0.999,
			"bore lip fillet: {v:?} wt={} torus_faces={} faces {}→{} center={center:?} axis={axis:?} major={major} minor={minor} \
			 band_off={max_off:.2e} removed_exact={removed_exact} (closed form {ring_true}) removed_tess={removed_tess} (bound {ring_bound}) \
			 bottom {vb:?} torus at {:?} axis {:?}",
			tessellate_default(&rounded).is_watertight(),
			torus_faces.len(),
			nut.face_count(),
			rounded.face_count(),
			bottom_torus.0,
			bottom_torus.1
		);
	}

	#[test]
	fn fillet_circular_rim_concave_honestly_rejects_out_of_scope_rims() {
		// The documented scope limits return None instead of a wrong solid: (1) no bore at
		// all; (2) a CONVEX boss rim — that is fillet_circular_rim's case, not this one;
		// (3) no room on the cap — the grown hole (bore_r + r = 8.5) would reach past the
		// hexagon's flats (apothem 8); (4) a bore shallower than the fillet radius (wall 2
		// < r = 2.5, while the grown hole 7.5 still clears the flats, isolating the depth
		// check as the rejector).
		let box_only = cuboid(DVec3::splat(-2.0), DVec3::splat(2.0));
		let boss = plate_with_boss(10.0, 3.0, 4.0, 6.0, 32);
		let nut = hex_nut_fixture(16.0, 8.0, 10.0);
		let shallow = hex_nut_fixture(16.0, 2.0, 10.0);
		assert!(
			fillet_circular_rim_concave(&box_only, DVec3::ZERO, 1.0, 8).is_none()
				&& fillet_circular_rim_concave(&boss, DVec3::new(0.0, 0.0, 9.0), 1.5, 8).is_none()
				&& fillet_circular_rim_concave(&nut, DVec3::new(0.0, 0.0, 9.0), 3.5, 8).is_none()
				&& fillet_circular_rim_concave(&shallow, DVec3::new(0.0, 0.0, 3.0), 2.5, 8).is_none(),
			"out-of-scope bore rims must be rejected, not mangled"
		);
	}

	#[test]
	fn chamfer_cylinder_rim_bevels_an_existing_cylinders_top_edge() {
		// In-place chamfer of an arbitrary primitive cylinder — the cut-edge mirror of the in-place
		// fillet. Valid genus-0 watertight solid with less material than the original.
		let cyl = cylinder(DVec3::new(-1.0, 4.0, 2.0), DVec3::Z, 5.0, 12.0, 48);
		let beveled = chamfer_cylinder_rim(&cyl, 2.0, 48).expect("a primitive cylinder rim chamfers");
		let v = validate(&beveled);
		assert!(
			v.closed && v.manifold && v.genus == 0 && tessellate_default(&beveled).is_watertight() && volume(&beveled).abs() < volume(&cyl).abs(),
			"in-place rim chamfer: {v:?} wt={} vol {} (orig {})",
			tessellate_default(&beveled).is_watertight(),
			volume(&beveled).abs(),
			volume(&cyl).abs()
		);
	}

	#[test]
	fn fillet_cylinder_rim_rounds_an_existing_cylinders_top_edge() {
		// The in-place rim fillet: take an arbitrary (off-origin) primitive cylinder and round its
		// top edge. The result must be a valid genus-0 watertight solid with less material than the
		// original sharp cylinder — detection (cylinder_rim) + robust revolve rebuild + pose.
		let cyl = cylinder(DVec3::new(3.0, -2.0, 1.0), DVec3::Z, 5.0, 12.0, 48);
		let rounded = fillet_cylinder_rim(&cyl, 2.0, 48, 8).expect("a primitive cylinder rim fillets");
		let v = validate(&rounded);
		assert!(
			v.closed
				&& v.manifold
				&& v.genus == 0
				&& tessellate_default(&rounded).is_watertight()
				&& volume(&rounded).abs() < volume(&cyl).abs(),
			"in-place rim fillet: {v:?} wt={} vol {} (orig {})",
			tessellate_default(&rounded).is_watertight(),
			volume(&rounded).abs(),
			volume(&cyl).abs()
		);
	}

	#[test]
	fn cylinder_rim_detects_the_top_circular_edge() {
		// Step 2 of the curved-edge fillet: detect the rim where a cylinder's wall meets its top
		// cap. A Ø10 cylinder 12 tall along +Z → top rim centred at (0,0,12), axis +Z, radius 5.
		let (r, h) = (5.0, 12.0);
		let cyl = cylinder(DVec3::ZERO, DVec3::Z, r, h, 48);
		let (center, axis, radius) = cylinder_rim(&cyl, DVec3::Z).expect("a cylinder has a top rim");
		assert!(
			(center - DVec3::new(0.0, 0.0, h)).length() < 1e-9 && (axis - DVec3::Z).length() < 1e-9 && (radius - r).abs() < 1e-9,
			"top rim: center {center:?} (want (0,0,{h})) axis {axis:?} radius {radius} (want {r})"
		);
	}

	#[test]
	fn cylinder_rim_centre_is_the_axis_not_the_cap_centroid() {
		// The centre is the cylinder axis ∩ cap plane, so an OFF-ORIGIN cylinder's rim centre lands
		// on its own axis (not the world origin): a Ø6 cylinder centred at x=10 → rim at (10,0,4).
		let cyl = cylinder(DVec3::new(10.0, 0.0, 0.0), DVec3::Z, 3.0, 4.0, 32);
		let (center, _, radius) = cylinder_rim(&cyl, DVec3::Z).expect("a cylinder has a top rim");
		assert!(
			(center - DVec3::new(10.0, 0.0, 4.0)).length() < 1e-9 && (radius - 3.0).abs() < 1e-9,
			"off-axis rim centre {center:?} (want (10,0,4)) radius {radius}"
		);
	}

	#[test]
	fn rim_fillet_band_lies_exactly_on_its_torus() {
		// The geometry core of a curved-edge (rim) fillet: a quarter-tube torus band, ring (major)
		// = 8, fillet radius (minor) = 2, axis +Z, centred at the origin. Every band point must lie
		// on that torus (its distance to the tube centre circle == minor); the ψ=0 edge sits on the
		// cylinder wall (radius major+minor = 10, on the centre plane), and the ψ=π/2 edge recedes
		// to radius major = 8, offset minor = 2 up the axis. (Standalone — not wired into round_edge.)
		let (surf, grid) = rim_fillet_band(DVec3::ZERO, DVec3::Z, 8.0, 2.0, 32, 6);
		let Surface::Torus { major, minor, .. } = surf else { panic!("expected a torus surface") };
		// Distance from p to the tube centre circle (radius `major` about +Z through the origin).
		let off = |p: DVec3| ((p.x * p.x + p.y * p.y).sqrt() - major).hypot(p.z) - minor;
		let max_off = grid.iter().flatten().fold(0.0_f64, |m, &p| m.max(off(p).abs()));
		let wall = grid[0][0];
		let cap = grid[0][6];
		assert!(
			max_off < 1e-9
				&& ((wall.x * wall.x + wall.y * wall.y).sqrt() - 10.0).abs() < 1e-9
				&& wall.z.abs() < 1e-9
				&& ((cap.x * cap.x + cap.y * cap.y).sqrt() - 8.0).abs() < 1e-9
				&& (cap.z - 2.0).abs() < 1e-9,
			"torus band off={max_off} wall={wall:?} cap={cap:?}"
		);
	}

	/// The persistent name of a box's +X∧+Y edge (faces 5 and 3 in cuboid order).
	fn px_py_edge() -> EdgeName {
		EdgeName::new(
			FaceName { operand: FaceSource::Primitive, source_face: 5 },
			FaceName { operand: FaceSource::Primitive, source_face: 3 },
		)
	}

	/// The (x, y) of a cylinder fillet face's axis — the corner the fillet rounded.
	fn fillet_axis_xy(s: &Solid) -> (f64, f64) {
		s.faces()
			.find_map(|f| match s.face(f).surface {
				Surface::Cylinder { origin, .. } => Some((origin.x, origin.y)),
				_ => None,
			})
			.expect("a cylinder fillet face")
	}

	#[test]
	fn fillet_rounds_a_named_box_edge_and_survives_an_edit() {
		let r = 0.3;
		let edge = px_py_edge();

		// Round the +X∧+Y edge of a unit box.
		let box1 = cuboid(DVec3::splat(-1.0), DVec3::splat(1.0));
		let f1 = fillet_edge(&box1, edge, r).expect("the named edge exists and is fillet-able");

		// It is a valid, watertight, genus-0 solid that has actually removed material
		// (a real round, not a no-op) and carries cylinder-tagged fillet faces.
		let v = validate(&f1);
		assert!(v.is_valid() && v.euler_characteristic == 2, "filleted box stays a valid genus-0 solid: {v:?}");
		assert!(tessellate_default(&f1).is_watertight(), "filleted box tessellates watertight");
		let removed = volume(&box1) - volume(&f1);
		let corner = r * r * 2.0; // L=2; upper bound = the whole square-corner prism
		assert!(removed > 1e-3 && removed < corner, "fillet removes a corner sliver, not nothing/too much (removed={removed})");
		// The fillet sits on the +X∧+Y corner: its cylinder axis is at (hi−r, hi−r).
		let (ax1, ay1) = fillet_axis_xy(&f1);
		assert!((ax1 - 0.7).abs() < 1e-9 && (ay1 - 0.7).abs() < 1e-9, "unit-box fillet axis at the +X+Y corner, got ({ax1},{ay1})");

		// LOAD-BEARING NAME: the SAME stored name re-resolves on a re-sized box and the
		// fillet re-attaches to the corresponding edge — the parametric-rebuild property.
		let box2 = cuboid(DVec3::splat(-2.0), DVec3::splat(2.0));
		assert_eq!(box2.edges_named(edge).len(), 1, "the name still resolves to one edge after the edit");
		let f2 = fillet_edge(&box2, edge, r).expect("the same name fillets the edited box");
		let v2 = validate(&f2);
		assert!(v2.is_valid() && v2.euler_characteristic == 2, "filleted edited box is a valid genus-0 solid: {v2:?}");
		assert!(tessellate_default(&f2).is_watertight(), "filleted edited box tessellates watertight");
		// DECISIVE: the fillet re-attached to the CORRECT edge of the EDITED box — its
		// axis moved from (0.7,0.7) to the bigger box's +X+Y corner (1.7,1.7). A name
		// that mis-resolved to another edge would round a different corner and fail here.
		let (ax2, ay2) = fillet_axis_xy(&f2);
		assert!((ax2 - 1.7).abs() < 1e-9 && (ay2 - 1.7).abs() < 1e-9, "edited-box fillet axis at the resized +X+Y corner, got ({ax2},{ay2})");
	}

	#[test]
	fn fillet_a_non_right_dihedral_edge_survives_an_edit() {
		use crate::build::extrude;
		use kernel_core::math::DVec2;
		// An equilateral-triangle prism: each vertical edge is a 60° convex dihedral
		// (na·nb = −cos60° = −0.5), NOT the 90° box case. Filleting one proves the
		// general-dihedral geometry. Faces: 0=bottom,1=top,2=side(P0→P1),3,4=side(P2→P0);
		// the edge at corner P0 is shared by side faces 2 and 4.
		let tri = |s: f64| vec![DVec2::new(0.0, 0.0), DVec2::new(2.0 * s, 0.0), DVec2::new(s, 3.0_f64.sqrt() * s)];
		let prism = |s: f64| extrude(&tri(s), 2.0);
		let edge = EdgeName::new(
			FaceName { operand: FaceSource::Primitive, source_face: 2 },
			FaceName { operand: FaceSource::Primitive, source_face: 4 },
		);

		let p1 = prism(1.0);
		assert_eq!(p1.edges_named(edge).len(), 1, "the 60° dihedral edge resolves to one edge");
		let r = 0.2;
		let f1 = fillet_edge(&p1, edge, r).expect("fillet a 60° dihedral edge");
		let v = validate(&f1);
		assert!(v.is_valid() && v.euler_characteristic == 2, "filleted prism valid genus-0: {v:?}");
		assert!(tessellate_default(&f1).is_watertight(), "filleted prism watertight");
		assert!(volume(&p1) - volume(&f1) > 1e-4, "the fillet removes material");

		// DECISIVE: the cylinder fillet face is tangent to BOTH side faces — its axis sits
		// at distance r from each side plane. This verifies the general C = vs − r(na+nb)/
		// (1+na·nb) tangency formula, not just that a cylinder was emitted.
		let axis_pt = f1
			.faces()
			.find_map(|fc| match f1.face(fc).surface {
				Surface::Cylinder { origin, .. } => Some(origin),
				_ => None,
			})
			.expect("a cylinder fillet face");
		let side_plane = |nm: FaceName| -> (DVec3, DVec3) {
			match f1.face(f1.faces_named(nm)[0]).surface {
				Surface::Plane { normal, origin } => (normal, origin),
				_ => panic!("side face is planar"),
			}
		};
		let (n2, o2) = side_plane(FaceName { operand: FaceSource::Primitive, source_face: 2 });
		let (n4, o4) = side_plane(FaceName { operand: FaceSource::Primitive, source_face: 4 });
		let d2 = (axis_pt - o2).dot(n2).abs();
		let d4 = (axis_pt - o4).dot(n4).abs();
		assert!((d2 - r).abs() < 1e-9 && (d4 - r).abs() < 1e-9, "cylinder tangent to both side faces (d2={d2}, d4={d4})");

		// Survives a parametric edit: scale the triangle; the same name re-resolves.
		let p2 = prism(1.5);
		assert_eq!(p2.edges_named(edge).len(), 1, "name re-resolves after scaling the prism");
		let f2 = fillet_edge(&p2, edge, r).expect("fillet the scaled prism edge");
		assert!(validate(&f2).is_valid() && tessellate_default(&f2).is_watertight(), "scaled filleted prism valid+watertight");
	}

	#[test]
	fn fillet_reports_typed_errors() {
		let box1 = cuboid(DVec3::splat(-1.0), DVec3::splat(1.0));
		let edge = px_py_edge();
		assert_eq!(fillet_edge(&box1, edge, 0.0).err(), Some(FilletError::BadRadius));
		assert_eq!(fillet_edge(&box1, edge, 5.0).err(), Some(FilletError::RadiusTooLarge));
		// A name no box edge bears (two opposite faces never share an edge).
		let bogus = EdgeName::new(
			FaceName { operand: FaceSource::Primitive, source_face: 4 },
			FaceName { operand: FaceSource::Primitive, source_face: 5 },
		);
		assert_eq!(fillet_edge(&box1, bogus, 0.3).err(), Some(FilletError::EdgeNotFound));
	}

	#[test]
	fn chamfer_bevels_a_named_box_edge_and_survives_an_edit() {
		let r = 0.3;
		let edge = px_py_edge();
		let box1 = cuboid(DVec3::splat(-1.0), DVec3::splat(1.0));
		let c1 = chamfer_edge(&box1, edge, r).expect("the named edge chamfers");

		let v = validate(&c1);
		assert!(v.is_valid() && v.euler_characteristic == 2, "chamfered box is a valid genus-0 solid: {v:?}");
		assert!(tessellate_default(&c1).is_watertight(), "chamfered box tessellates watertight");
		assert!(volume(&box1) - volume(&c1) > 1e-3, "the chamfer removes a corner wedge");

		// A chamfer is FLAT: exactly one diagonal bevel plane (normal ≈ (X+Y)/√2), and —
		// unlike a fillet — no cylindrical faces at all.
		let inv_sqrt2 = std::f64::consts::FRAC_1_SQRT_2;
		assert!(
			c1.faces().any(|f| matches!(c1.face(f).surface,
				Surface::Plane { normal, .. }
				if (normal.x - inv_sqrt2).abs() < 1e-6 && (normal.y - inv_sqrt2).abs() < 1e-6 && normal.z.abs() < 1e-6)),
			"the chamfer adds a diagonal bevel plane"
		);
		assert!(
			!c1.faces().any(|f| matches!(c1.face(f).surface, Surface::Cylinder { .. })),
			"a chamfer has no cylindrical faces"
		);

		// The same stored name re-resolves and chamfers the resized box.
		let box2 = cuboid(DVec3::splat(-2.0), DVec3::splat(2.0));
		let c2 = chamfer_edge(&box2, edge, r).expect("the same name chamfers the edited box");
		assert!(validate(&c2).is_valid() && tessellate_default(&c2).is_watertight(), "chamfered edited box valid+watertight");
	}

	#[test]
	fn witness_disambiguates_a_split_named_edge() {
		use crate::booleans::union;
		// SPLIT-FRAGMENT DISAMBIGUATION. A small box unioned onto the MIDDLE of A's
		// +X∧+Y edge leaves that edge only at the two ends — two collinear fragments,
		// BOTH named {OperandA:+X(5), OperandA:+Y(3)}. The bare name is genuinely
		// ambiguous; a witness point resolves it to the intended fragment.
		let a = cuboid(DVec3::splat(-2.0), DVec3::splat(2.0));
		let b = cuboid(DVec3::new(1.0, 1.0, -0.5), DVec3::new(3.0, 3.0, 0.5));
		let u = union(&a, &b);
		let edge = EdgeName::new(
			FaceName { operand: FaceSource::OperandA, source_face: 5 },
			FaceName { operand: FaceSource::OperandA, source_face: 3 },
		);
		assert_eq!(u.edges_named(edge).len(), 2, "the union splits the named edge into two same-named fragments");
		assert_eq!(fillet_edge(&u, edge, 0.3).err(), Some(FilletError::EdgeAmbiguous), "the bare name cannot choose");

		// The resolver picks the geometrically-nearest fragment — the core of the
		// disambiguation. Distinct witnesses select distinct fragments, each the one
		// nearest the witness (upper z>0, lower z<0).
		let mid_z = |e: crate::topo::EdgeId| {
			let he = *u.half_edge(u.edge(e).half_edge);
			(u.position(he.origin).z + u.position(u.half_edge(he.next).origin).z) * 0.5
		};
		let e_up = nearest_named_edge(&u, edge, DVec3::new(2.0, 2.0, 1.5)).unwrap();
		let e_lo = nearest_named_edge(&u, edge, DVec3::new(2.0, 2.0, -1.5)).unwrap();
		assert_ne!(e_up, e_lo, "distinct witnesses resolve to distinct fragments");
		assert!(mid_z(e_up) > 0.0 && mid_z(e_lo) < 0.0, "each witness selects the fragment nearest it");

		// HONEST LIMITATION (surfaced, not hidden): a split fragment's split-point endpoint
		// is high-valence (a feature junction), which the trivalent corner-rebuild does not
		// yet handle. The geometry returns a clean typed Unsupported — never a broken solid —
		// AND it is no longer Ambiguous, proving the witness DID resolve to one fragment.
		// Completing a split-fragment fillet needs high-valence-endpoint (local-edit) support.
		assert_eq!(
			fillet_edge_near(&u, edge, 0.3, DVec3::new(2.0, 2.0, 1.5)).err(),
			Some(FilletError::Unsupported),
			"resolved past ambiguity to one fragment; geometry honestly reports the high-valence limit"
		);
	}

	#[test]
	fn fillet_a_boolean_result_edge_survives_an_edit() {
		use crate::booleans::union;
		// Filleting generalises PAST primitives: round a convex edge of a UNION whose
		// faces are OPERAND-named, not Primitive. In union(A=[-s,s]³, B=[0,4]³) operand
		// A's far corner edge (A's −X ∧ −Y faces, cuboid indices 4 and 2) survives whole
		// and is convex. The Operand-based EdgeName must re-resolve after editing A's size.
		let edge = EdgeName::new(
			FaceName { operand: FaceSource::OperandA, source_face: 4 },
			FaceName { operand: FaceSource::OperandA, source_face: 2 },
		);
		let b = cuboid(DVec3::ZERO, DVec3::splat(4.0));
		let make = |s: f64| union(&cuboid(DVec3::splat(-s), DVec3::splat(s)), &b);

		let u1 = make(2.0);
		assert_eq!(u1.edges_named(edge).len(), 1, "the operand-named convex edge resolves to one edge");
		// The edge is genuinely on boolean-result topology: its faces are Operand-named.
		let e = u1.edges_named(edge)[0];
		assert_eq!(u1.edge_name(e).map(|n| n.faces[0].operand), Some(FaceSource::OperandA), "edge faces are operand-named");

		let f1 = fillet_edge(&u1, edge, 0.5).expect("fillet a boolean-result edge");
		assert!(validate(&f1).is_valid() && tessellate_default(&f1).is_watertight(), "filleted union valid+watertight: {:?}", validate(&f1));
		let (x1, y1) = fillet_axis_xy(&f1);
		assert!((x1 + 1.5).abs() < 1e-9 && (y1 + 1.5).abs() < 1e-9, "fillet at A's −X−Y corner (−1.5,−1.5), got ({x1},{y1})");

		// EDIT operand A larger; the same Operand-based name re-resolves and the fillet
		// re-attaches — its axis moves with the grown corner (−1.5,−1.5) → (−2.5,−2.5).
		let u2 = make(3.0);
		assert_eq!(u2.edges_named(edge).len(), 1, "name re-resolves after growing operand A");
		let f2 = fillet_edge(&u2, edge, 0.5).expect("fillet the edited union");
		assert!(validate(&f2).is_valid() && tessellate_default(&f2).is_watertight(), "edited filleted union valid+watertight: {:?}", validate(&f2));
		let (x2, y2) = fillet_axis_xy(&f2);
		assert!((x2 + 2.5).abs() < 1e-9 && (y2 + 2.5).abs() < 1e-9, "fillet re-attached to grown corner (−2.5,−2.5), got ({x2},{y2})");
	}
}
