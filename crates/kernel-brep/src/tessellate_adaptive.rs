// Copyright (c) LMCAD. Licensed under the MIT License.

//! Crack-free adaptive B-rep → triangle [`Mesh`] tessellation.
//!
//! The default [`crate::tessellate`] forces `curved_subdivisions = 1` because
//! subdividing a curved face independently of its neighbours introduces
//! T-junctions: a curved/planar shared edge would gain extra points on the
//! curved side that the planar side does not know about, leaving a hairline
//! crack the weld cannot close.
//!
//! This module removes that limitation by making *shared-edge point identity*
//! the central invariant. Each topological [`EdgeId`] is subdivided exactly
//! once into a polyline whose interior points are projected onto a chosen
//! analytic surface; **both** faces incident to the edge consume that same
//! polyline (reversed for the second half-edge). Because the two faces share
//! the identical floating-point positions along their common edge, the boundary
//! polylines match seam-for-seam and the welded mesh is watertight even with
//! many subdivisions.
//!
//! Algorithm:
//! 1. Pick a projection surface per edge — the curved incident face if any
//!    (so the seam follows the true curve); if both faces are planar the edge
//!    is straight and no interior points are added.
//! 2. Subdivide each edge into `edge_segments` segments, projecting the
//!    linearly interpolated endpoints onto the chosen surface. Store the
//!    polyline keyed by [`EdgeId`].
//! 3. Assemble each face boundary from its half-edges' shared polylines,
//!    reversing a half-edge's polyline when it is the *second* half-edge of its
//!    edge so direction is respected.
//! 4. Triangulate: planar faces are ear-clipped in-plane (same approach as
//!    [`crate::tessellate`]); curved faces triangulate the dense boundary plus
//!    interior grid points, all snapped onto the face surface.
//! 5. Weld duplicated boundary vertices into a shared-vertex manifold.

use kernel_core::math::{DVec2, DVec3};
use kernel_core::mesh::Mesh;

use crate::geom::{perp_basis, Surface, SurfaceChart};
use crate::topo::{EdgeId, FaceId, HalfEdgeId, Solid};

/// Weld tolerance used after assembling all faces. Matches the default in
/// [`crate::tessellate::TessOptions`].
const WELD_TOLERANCE: f32 = 1e-5;

/// Tessellate a solid crack-free with `edge_segments` segments per topological
/// edge. `edge_segments == 1` reproduces the control-facet behaviour of
/// [`crate::tessellate_default`]; higher values smooth curved faces while
/// keeping curved/planar seams coincident, so the result stays watertight.
pub fn tessellate_adaptive(solid: &Solid, edge_segments: usize) -> Mesh {
	let segs = edge_segments.max(1);
	let edge_points = build_edge_points(solid, segs);

	let mut mesh = Mesh::new();
	for f in solid.faces() {
		let surface = solid.face(f).surface;
		let boundary = face_boundary(solid, f, &edge_points);
		if boundary.len() < 3 {
			continue;
		}
		// Winding is taken from the topological loop (already outward-oriented),
		// never from a surface tag whose stored normal sign may be incidental.
		let outward = newell_normal(&boundary);
		match surface {
			Surface::Plane { .. } => tessellate_planar(&mut mesh, &boundary, outward),
			curved => tessellate_curved(&mut mesh, &boundary, curved, segs, outward),
		}
	}
	mesh.weld(WELD_TOLERANCE);
	mesh
}

/// Crack-free tessellation with the edge subdivision chosen automatically so every
/// curved face stays within `tol` (model units) of its true surface.
///
/// The most-curved face sets a single subdivision density that is applied
/// uniformly, so shared-edge identity — and hence watertightness — is preserved.
/// (A *per-face* adaptive count cannot: a denser curved face would leave T-junctions
/// against a coarser neighbour. That is exactly why the simpler per-face
/// [`crate::tessellate`] forces `curved_subdivisions = 1`.) A larger model or a
/// tighter `tol` raises the density; a flat or already-fine solid stays at one
/// segment.
pub fn tessellate_adaptive_tol(solid: &Solid, tol: f64) -> Mesh {
	let mut segs = 1usize;
	if tol > 0.0 {
		for f in solid.faces() {
			let surface = solid.face(f).surface;
			if matches!(surface, Surface::Plane { .. }) {
				continue;
			}
			// Sagitta of the control facet: its corners lie on the surface but its
			// edge midpoints and centroid do not, so |signed_value| there is the
			// chord error. Subdividing each edge into n shrinks it ≈ n², so n ≈ √(s/tol).
			let poly = solid.face_polygon(f);
			if poly.len() < 3 {
				continue;
			}
			// A MERGED wide-span face self-refines with interior points (see
			// tessellate.rs); its centroid sagitta is span-scale and would drive
			// the shared-edge density to the clamp for no fidelity gain — skip it.
			let newell = newell_normal(&poly);
			if crate::tessellate::merged_curved_ring(&poly, &surface, newell) {
				continue;
			}
			let centroid = poly.iter().fold(DVec3::ZERO, |a, &p| a + p) / poly.len() as f64;
			let mut sagitta = surface.signed_value(centroid).abs();
			for i in 0..poly.len() {
				let mid = (poly[i] + poly[(i + 1) % poly.len()]) * 0.5;
				sagitta = sagitta.max(surface.signed_value(mid).abs());
			}
			if sagitta > tol {
				segs = segs.max((sagitta / tol).sqrt().ceil() as usize);
			}
		}
	}
	tessellate_adaptive(solid, segs.clamp(1, 128))
}

// --- Shared edge points ------------------------------------------------------

/// Per-edge subdivision polylines, indexed by `EdgeId.0`. Each entry runs from
/// the origin of the edge's *canonical* half-edge ([`crate::topo::Edge::half_edge`])
/// to the origin of that half-edge's `next`, inclusive of both endpoints.
struct EdgePoints {
	/// `polylines[e]` are the `segs + 1` points of edge `e` (or just the two
	/// endpoints when the edge is straight).
	polylines: Vec<Vec<DVec3>>,
}

impl EdgePoints {
	/// Points of an edge oriented for `he`: forward if `he` is the canonical
	/// half-edge of its edge, reversed otherwise.
	fn for_half_edge(&self, solid: &Solid, he: HalfEdgeId) -> Vec<DVec3> {
		let edge_id = solid.half_edge(he).edge;
		let canonical = solid.edge(edge_id).half_edge;
		let pts = &self.polylines[edge_id.0 as usize];
		if he == canonical {
			pts.clone()
		} else {
			pts.iter().rev().copied().collect()
		}
	}
}

/// For each edge choose a projection surface and subdivide once. Both faces
/// incident to the edge then reuse the identical points, preventing cracks.
fn build_edge_points(solid: &Solid, segs: usize) -> EdgePoints {
	let mut polylines: Vec<Vec<DVec3>> = Vec::with_capacity(solid.edge_count());
	for e in 0..solid.edge_count() as u32 {
		let edge_id = EdgeId(e);
		let he = solid.edge(edge_id).half_edge;
		let start = solid.position(solid.half_edge(he).origin);
		let next_he = solid.half_edge(he).next;
		let end = solid.position(solid.half_edge(next_he).origin);

		let proj_surface = edge_projection_surface(solid, edge_id);
		polylines.push(subdivide_edge(start, end, proj_surface, segs));
	}
	EdgePoints { polylines }
}

/// The surface to project an edge's interior points onto: the curved incident
/// face if one exists; `None` (a straight edge) when both faces are planar.
/// When both faces are curved (e.g. sphere–sphere) either works; we take the
/// canonical half-edge's face.
fn edge_projection_surface(solid: &Solid, edge_id: EdgeId) -> Option<Surface> {
	let he = solid.edge(edge_id).half_edge;
	let face_a = solid.half_edge(he).face;
	let surf_a = solid.face(face_a).surface;
	let twin = solid.half_edge(he).twin;
	let surf_b = twin.map(|t| solid.face(solid.half_edge(t).face).surface);

	let is_planar = |s: &Surface| matches!(s, Surface::Plane { .. });
	match (is_planar(&surf_a), surf_b) {
		// `a` curved: use it (the seam should follow the true curve of `a`).
		(false, _) => Some(surf_a),
		// `a` planar, `b` curved: use `b`.
		(true, Some(b)) if !is_planar(&b) => Some(b),
		// Both planar (or boundary edge with no twin): straight edge.
		_ => None,
	}
}

/// Subdivide a straight chord into `segs` segments. When `surface` is `Some`,
/// each interior point is projected onto it so it lands on the true curve;
/// otherwise only the two endpoints are returned (a straight edge needs no
/// interior points and adding them would only risk drift off the line).
fn subdivide_edge(start: DVec3, end: DVec3, surface: Option<Surface>, segs: usize) -> Vec<DVec3> {
	match surface {
		None => vec![start, end],
		Some(s) => {
			let mut out = Vec::with_capacity(segs + 1);
			out.push(start);
			for k in 1..segs {
				let t = k as f64 / segs as f64;
				out.push(s.project(start.lerp(end, t)));
			}
			out.push(end);
			out
		}
	}
}

/// Assemble a face's dense boundary polyline from its outer loop. Each
/// half-edge contributes its edge's shared points (direction-corrected),
/// dropping the last point so the seam to the next half-edge is not duplicated.
fn face_boundary(solid: &Solid, f: FaceId, edge_points: &EdgePoints) -> Vec<DVec3> {
	let outer = solid.face(f).outer;
	let mut boundary = Vec::new();
	for he in solid.loop_half_edges(outer) {
		let pts = edge_points.for_half_edge(solid, he);
		// `pts` runs origin→next-origin for this half-edge. Append all but the
		// final point; the next half-edge starts exactly there.
		for &p in &pts[..pts.len().saturating_sub(1)] {
			boundary.push(p);
		}
	}
	boundary
}

// --- Normals / winding -------------------------------------------------------

/// Newell's area-weighted polygon normal (winding-following).
fn newell_normal(poly: &[DVec3]) -> DVec3 {
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

/// Push a triangle with per-vertex normals, forcing the winding so the geometric
/// normal agrees with `outward`.
#[allow(clippy::too_many_arguments)] // a triangle's 3 verts + 3 normals + outward ref
fn push_tri(mesh: &mut Mesh, a: DVec3, b: DVec3, c: DVec3, na: DVec3, nb: DVec3, nc: DVec3, outward: DVec3) {
	let geo = (b - a).cross(c - a);
	let (b, c, nb, nc) = if geo.dot(outward) < 0.0 { (c, b, nc, nb) } else { (b, c, nb, nc) };
	let base = mesh.positions.len() as u32;
	for (p, n) in [(a, na), (b, nb), (c, nc)] {
		mesh.positions.push(p.as_vec3());
		mesh.normals.push(n.as_vec3());
	}
	mesh.indices.extend_from_slice(&[base, base + 1, base + 2]);
}

// --- Planar faces: ear clipping ----------------------------------------------

/// Ear-clip a (densely sampled) planar boundary polygon in its own plane. Delegates to
/// the crate's robust ear-clipper (exact `orient2d` corner test + collinear-drain on
/// stall), so a boolean's keyhole-bridged annular cap — a hole stitched into the outer
/// loop by a zero-width corridor — clips watertight instead of into overlapping triangles.
fn tessellate_planar(mesh: &mut Mesh, poly: &[DVec3], normal: DVec3) {
	if poly.len() < 3 {
		return;
	}
	let (u, v) = perp_basis(normal);
	let p2: Vec<DVec2> = poly.iter().map(|p| DVec2::new(p.dot(u), p.dot(v))).collect();
	crate::tessellate::ear_clip_ring(mesh, poly, &p2, (0..poly.len()).collect(), normal);
}

// --- Curved faces: surface-snapped grid --------------------------------------

/// Triangulate a curved face. The boundary polyline already carries the dense,
/// shared seam points; interior grid points are generated by bilinear /
/// barycentric blends of the *original* topological corners and snapped onto
/// the surface, exactly as `tessellate::tessellate_curved` does. The boundary
/// rings are stitched to the interior grid so that the seam uses the shared
/// edge points verbatim (no T-junctions), while the interior stays smooth.
fn tessellate_curved(mesh: &mut Mesh, boundary: &[DVec3], surface: Surface, segs: usize, face_outward: DVec3) {
	// Vertex normal from the analytic surface, sign-corrected to the face's
	// outward direction; winding always uses `face_outward`.
	let nrm = |p: DVec3| {
		let n = surface.normal_at(p);
		if n.dot(face_outward) < 0.0 {
			-n
		} else {
			n
		}
	};

	// Recover the topological corners: a corner is where a new half-edge begins,
	// i.e. every `segs`-th boundary sample (the boundary was built by appending
	// `segs` points per edge). With one edge per side, corners are evenly spaced.
	let corners = recover_corners(boundary, segs);

	match corners.len() {
		4 => tessellate_curved_quad(mesh, boundary, &corners, surface, segs, face_outward, &nrm),
		3 => tessellate_curved_tri(mesh, boundary, &corners, surface, segs, face_outward, &nrm),
		_ => {
			if boundary.len() < 3 {
				return;
			}
			// A MERGED wide-span curved face (a recover-pass chart face) is
			// triangulated with interior refinement — the dense boundary is
			// consumed verbatim (seam-shared, crack-free) and interior points
			// restore the bulge a boundary-only clip would lose. Same routine as
			// `tessellate_default` (see the tessellate.rs module doc).
			if crate::tessellate::merged_curved_ring(boundary, &surface, face_outward)
				&& push_refined(mesh, boundary, &surface, face_outward)
			{
				return;
			}
			// A ring WARPED off its plane (seam-snapped vertices on the true
			// intersection curve) ear-clips in the surface's PARAMETER SPACE: a
			// centroid fan of a warped non-convex ring can fold, and the boundary
			// points are already the shared seam samples so a boundary-only clip
			// stays crack-free.
			if let Some(p2) = SurfaceChart::for_warped_ring(&surface, boundary, face_outward).and_then(|c| c.uv_ring(boundary)) {
				crate::tessellate::ear_clip_ring(mesh, boundary, &p2, (0..boundary.len()).collect(), face_outward);
				return;
			}
			// Generic curved polygon: fan the dense boundary from its projected
			// centroid. The boundary points are already shared, so the seam is
			// crack-free; the interior fan is a reasonable approximation.
			let centroid: DVec3 = boundary.iter().copied().sum::<DVec3>() / boundary.len() as f64;
			let center = surface.project(centroid);
			let n = boundary.len();
			for k in 0..n {
				let a = boundary[k];
				let b = boundary[(k + 1) % n];
				push_tri(mesh, center, a, b, nrm(center), nrm(a), nrm(b), face_outward);
			}
		}
	}
}

/// Push a merged face's interior-refined triangulation (see
/// [`crate::tessellate::refine_curved_ring`]) with true per-vertex surface
/// normals, each triangle wound against the surface normal at its own centroid
/// (a wide-span face's outward direction varies across the face). The outward
/// SIGN comes from the ring's Newell normal. `false` (nothing pushed) when the
/// ring cannot be refined — the caller falls back to the boundary-only paths.
fn push_refined(mesh: &mut Mesh, boundary: &[DVec3], surface: &Surface, newell: DVec3) -> bool {
	let Some((pts, tris, outward)) = crate::tessellate::refine_curved_ring(boundary, surface) else {
		return false;
	};
	crate::tessellate::push_refined_tris(mesh, &pts, &tris, &outward, boundary, surface, newell);
	true
}

/// Indices into `boundary` of the topological corners. Each face side
/// contributes `segs` boundary samples (the side's leading point plus its
/// interior points), so corners are at `0, segs, 2*segs, …`.
fn recover_corners(boundary: &[DVec3], segs: usize) -> Vec<usize> {
	let n = boundary.len();
	if segs == 0 || !n.is_multiple_of(segs) {
		// Fallback: treat every sample as a corner (e.g. mixed straight/curved
		// sides of differing density). The polygon fan path handles this.
		return (0..n).collect();
	}
	(0..n / segs).map(|s| s * segs).collect()
}

/// Index of the boundary sample at offset `o` along the side starting at corner
/// `c` (0..=segs, wrapping at the end of the boundary).
#[inline]
fn side_sample(boundary_len: usize, corner_start: usize, offset: usize) -> usize {
	(corner_start + offset) % boundary_len
}

/// Bilinearly blend the four corners, snap to the surface. `(s, t)` in `[0, 1]`.
fn quad_point(corners: &[DVec3], surface: Surface, s: f64, t: f64) -> DVec3 {
	let a = corners[0].lerp(corners[1], s);
	let b = corners[3].lerp(corners[2], s);
	surface.project(a.lerp(b, t))
}

/// Quad curved face: build a `(segs+1)²` grid whose four borders are taken
/// verbatim from the shared boundary polyline and whose interior is the
/// surface-snapped bilinear blend.
#[allow(clippy::too_many_arguments)]
fn tessellate_curved_quad(
	mesh: &mut Mesh,
	boundary: &[DVec3],
	corners: &[usize],
	surface: Surface,
	segs: usize,
	face_outward: DVec3,
	nrm: &impl Fn(DVec3) -> DVec3,
) {
	let n = segs;
	let bl = boundary.len();
	// Corner positions in boundary order: c0→c1 (side 0), c1→c2 (side 1),
	// c2→c3 (side 2), c3→c0 (side 3).
	let corner_pos: Vec<DVec3> = corners.iter().map(|&i| boundary[i]).collect();

	let mut grid: Vec<Vec<DVec3>> = (0..=n)
		.map(|i| {
			let s = i as f64 / n as f64;
			(0..=n)
				.map(|j| {
					let t = j as f64 / n as f64;
					quad_point(&corner_pos, surface, s, t)
				})
				.collect()
		})
		.collect();

	// Overwrite the four borders with the exact shared boundary samples so the
	// seam matches the neighbour faces. Indexing: grid[i][j], side 0 (i: 0→n at
	// j=0) is c0→c1, side 1 (j: 0→n at i=n) is c1→c2, side 2 (i: n→0 at j=n) is
	// c2→c3, side 3 (j: n→0 at i=0) is c3→c0.
	for off in 0..=n {
		// Side 0: corner 0 to corner 1, varying i, j = 0.
		grid[off][0] = boundary[side_sample(bl, corners[0], off)];
		// Side 1: corner 1 to corner 2, i = n, varying j.
		grid[n][off] = boundary[side_sample(bl, corners[1], off)];
		// Side 2: corner 2 to corner 3, varying i (n→0), j = n.
		grid[n - off][n] = boundary[side_sample(bl, corners[2], off)];
		// Side 3: corner 3 to corner 0, i = 0, varying j (n→0).
		grid[0][n - off] = boundary[side_sample(bl, corners[3], off)];
	}

	for i in 0..n {
		for j in 0..n {
			let p00 = grid[i][j];
			let p10 = grid[i + 1][j];
			let p11 = grid[i + 1][j + 1];
			let p01 = grid[i][j + 1];
			push_tri(mesh, p00, p10, p11, nrm(p00), nrm(p10), nrm(p11), face_outward);
			push_tri(mesh, p00, p11, p01, nrm(p00), nrm(p11), nrm(p01), face_outward);
		}
	}
}

/// Triangle curved face: barycentric grid with the three borders taken verbatim
/// from the shared boundary polyline.
#[allow(clippy::too_many_arguments)]
fn tessellate_curved_tri(
	mesh: &mut Mesh,
	boundary: &[DVec3],
	corners: &[usize],
	surface: Surface,
	segs: usize,
	face_outward: DVec3,
	nrm: &impl Fn(DVec3) -> DVec3,
) {
	let n = segs;
	let bl = boundary.len();
	let (a, b, c) = (boundary[corners[0]], boundary[corners[1]], boundary[corners[2]]);

	// `grid[i][j]` for i + j <= n, barycentric (1 - bi - bj, bi over a→b, bj over a→c).
	let pt = |i: usize, j: usize| {
		let bi = i as f64 / n as f64;
		let bj = j as f64 / n as f64;
		surface.project(a + (b - a) * bi + (c - a) * bj)
	};
	let mut grid: Vec<Vec<DVec3>> = (0..=n).map(|i| (0..=(n - i)).map(|j| pt(i, j)).collect()).collect();

	// Overwrite the three edges with shared samples.
	// Edge a→b: corners[0]→corners[1], grid[i][0], i = 0..=n.
	// Edge b→c: corners[1]→corners[2], grid[n - k][k], k = 0..=n.
	// Edge c→a: corners[2]→corners[0], grid[0][n - k], k = 0..=n.
	for off in 0..=n {
		grid[off][0] = boundary[side_sample(bl, corners[0], off)];
		grid[n - off][off] = boundary[side_sample(bl, corners[1], off)];
		grid[0][n - off] = boundary[side_sample(bl, corners[2], off)];
	}

	for i in 0..n {
		for j in 0..(n - i) {
			let p0 = grid[i][j];
			let p1 = grid[i + 1][j];
			let p2 = grid[i][j + 1];
			push_tri(mesh, p0, p1, p2, nrm(p0), nrm(p1), nrm(p2), face_outward);
			if i + j + 2 <= n {
				let p3 = grid[i + 1][j + 1];
				push_tri(mesh, p1, p3, p2, nrm(p1), nrm(p3), nrm(p2), face_outward);
			}
		}
	}
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::build::{cuboid, cylinder, sphere};
	use crate::tessellate::tessellate_default;
	use kernel_core::math::DVec3;
	use std::f64::consts::PI;

	/// Total stored points across every edge polyline (used to assert that
	/// straight edges add no interior points).
	fn shared_edge_point_count(solid: &Solid, segs: usize) -> usize {
		let ep = build_edge_points(solid, segs);
		(0..solid.edge_count()).map(|e| ep.polylines[e].len()).sum()
	}

	#[test]
	fn box_is_exact_and_watertight() {
		let solid = cuboid(DVec3::new(-1.0, -1.0, -1.0), DVec3::new(1.0, 1.0, 1.0));
		let mesh = tessellate_adaptive(&solid, 4);
		// A box has only planar faces; adaptive tessellation must reproduce the
		// exact volume and stay watertight.
		assert!(mesh.is_watertight(), "box mesh must be watertight");
		assert!((mesh.signed_volume().abs() - 8.0).abs() < 1e-9, "box volume must be exactly 8");
		// Straight edges add no interior points: 12 edges × 2 endpoints.
		assert_eq!(shared_edge_point_count(&solid, 4), 12 * 2);
	}

	#[test]
	fn cylinder_is_watertight_and_more_accurate() {
		// Coarse construction so the closed-form error is visible.
		let radius = 1.0;
		let height = 2.0;
		let segments = 8;
		let solid = cylinder(DVec3::ZERO, DVec3::Z, radius, height, segments);

		let adaptive = tessellate_adaptive(&solid, 4);
		assert!(adaptive.is_watertight(), "cylinder adaptive mesh must be watertight");

		let exact = PI * radius * radius * height;
		let v_default = tessellate_default(&solid).signed_volume().abs();
		let v_adaptive = adaptive.signed_volume().abs();
		// Adaptive subdivision of the side faces (projected onto the cylinder)
		// must approach the true volume more closely than the control facets.
		assert!(
			(v_adaptive - exact).abs() < (v_default - exact).abs(),
			"adaptive cylinder volume {v_adaptive} must beat default {v_default} (exact {exact})",
		);
	}

	#[test]
	fn sphere_is_watertight_and_more_accurate() {
		let radius = 1.0;
		let solid = sphere(DVec3::ZERO, radius, 8, 6);

		let adaptive = tessellate_adaptive(&solid, 4);
		assert!(adaptive.is_watertight(), "sphere adaptive mesh must be watertight");

		let exact = 4.0 / 3.0 * PI * radius * radius * radius;
		let v_default = tessellate_default(&solid).signed_volume().abs();
		let v_adaptive = adaptive.signed_volume().abs();
		assert!(
			(v_adaptive - exact).abs() < (v_default - exact).abs(),
			"adaptive sphere volume {v_adaptive} must beat default {v_default} (exact {exact})",
		);
	}

	#[test]
	fn segments_one_matches_default_topology() {
		// With edge_segments = 1 the adaptive path adds no interior points, so it
		// must still be watertight (a regression guard for the seam stitching).
		let solid = cylinder(DVec3::ZERO, DVec3::Z, 1.0, 2.0, 12);
		let mesh = tessellate_adaptive(&solid, 1);
		assert!(mesh.is_watertight(), "edge_segments=1 must be watertight");
	}
}
