// Copyright (c) LMCAD. Licensed under the MIT License.

//! Freeform NURBS surfacing — building tensor-product [`NurbsSurface`]s from
//! input curves rather than analytic primitives.
//!
//! Two general constructions are provided, both reducing to *skinning* a surface
//! through a stack of section polylines:
//!
//! - [`loft`] skins a surface through `N` section polylines (each a row of `M`
//!   control points). The `M` points become the `u` control rows and the `N`
//!   profiles the `v` direction, with open-uniform clamped knot vectors in both
//!   parameters. This interpolates each profile as an iso-`v` control polygon and
//!   B-spline-blends between them along `v`.
//! - [`sweep`] translates a single profile polyline to every point of a path
//!   polyline (optionally re-orienting it to follow the path tangent via a
//!   parallel-transport frame), then [`loft`]s through those copies.
//!
//! These are *control-net* constructions: the input polyline points are used
//! directly as control points (weights all 1, i.e. a non-rational B-spline). The
//! surface therefore passes exactly through the corner control points of the
//! first/last rows and columns (clamped knots) and blends smoothly in between —
//! which is the standard, shape-agnostic behaviour of a skinned NURBS surface.
//!
//! Everything is general: any consistent set of profiles / any path works, with
//! graceful fallbacks for degenerate inputs (collapsed profiles, too-short
//! polylines, mismatched lengths).

use kernel_core::math::{DVec2, DVec3};
use kernel_core::mesh::Mesh;

use crate::curved_boolean::{boundary_loops, trim_mesh_by_surface, Keep};
use crate::geom::{perp_basis, Surface};
use crate::nurbs::{FreeformFace, NurbsSurface};
use crate::topo::{FaceInput, Solid};

/// Build an open-uniform *clamped* knot vector for `n_ctrl` control points of the
/// given `degree`.
///
/// Layout: `degree + 1` repeated zeros, then ascending interior knots `1..=k`,
/// then `degree + 1` repeats of the maximum. This clamps the curve to its first
/// and last control points and yields exactly `n_ctrl + degree + 1` knots, which
/// is the invariant [`NurbsSurface::new`] checks.
fn open_uniform_knots(n_ctrl: usize, degree: usize) -> Vec<f64> {
	let n_knots = n_ctrl + degree + 1;
	// Interior (non-clamp) knot count. With n_ctrl > degree this is >= 0.
	let interior = n_knots.saturating_sub(2 * (degree + 1));
	let mut knots = Vec::with_capacity(n_knots);
	knots.resize(degree + 1, 0.0); // clamp at the start
	for i in 1..=interior {
		knots.push(i as f64);
	}
	let max = (interior + 1) as f64;
	knots.resize(knots.len() + degree + 1, max); // clamp at the end
	knots
}

/// Effective degree usable for a control axis of length `n`: at least 1 (so a
/// genuine B-spline span exists) but never `>= n` (which [`NurbsSurface::new`]
/// rejects, since it needs `n > degree`).
///
/// `requested` is the caller's desired degree; it is clamped down to `n - 1` and
/// up to `1`. For a degenerate axis of length `1` this returns `0` (a single
/// constant control point), which the caller handles by duplication.
fn clamp_degree(requested: usize, n: usize) -> usize {
	if n <= 1 {
		return 0;
	}
	requested.clamp(1, n - 1)
}

/// Skin a NURBS surface through `N` section polylines.
///
/// Each entry of `profiles` is one section: a row of `M` control points. **All
/// profiles must have the same point count `M`.** The resulting surface uses the
/// `M` points of each profile as its `u` control rows and the `N` profiles as its
/// `v` direction:
///
/// - `degree_u = min(3, M - 1)` (cubic where there is room, lower for short
///   profiles) with an open-uniform clamped `u` knot vector;
/// - `degree_v` is the requested value, clamped to `[1, N - 1]`, with an
///   open-uniform clamped `v` knot vector.
///
/// All weights are `1` (a non-rational B-spline skin). The control grid is stored
/// `control[i][j]` with `i` over `u` (the `M` profile points) and `j` over `v`
/// (the `N` profiles), matching [`NurbsSurface`]'s indexing.
///
/// Returns `None` for degenerate input: fewer than two profiles, fewer than two
/// points per profile, mismatched profile lengths, or any non-finite coordinate.
pub fn loft(profiles: &[Vec<DVec3>], degree_v: usize) -> Option<NurbsSurface> {
	let n_v = profiles.len();
	if n_v < 2 {
		return None;
	}
	let n_u = profiles[0].len();
	if n_u < 2 {
		return None;
	}
	// All profiles must share the same point count, and all coordinates finite.
	for profile in profiles {
		if profile.len() != n_u {
			return None;
		}
		for p in profile {
			if !p.is_finite() {
				return None;
			}
		}
	}

	let deg_u = clamp_degree(3, n_u).max(1);
	let deg_v = clamp_degree(degree_v, n_v).max(1);

	// control[i][j]: i over u (profile points 0..n_u), j over v (profiles 0..n_v).
	let mut control: Vec<Vec<DVec3>> = vec![vec![DVec3::ZERO; n_v]; n_u];
	let mut weights: Vec<Vec<f64>> = vec![vec![1.0; n_v]; n_u];
	for (j, profile) in profiles.iter().enumerate() {
		for (i, &p) in profile.iter().enumerate() {
			control[i][j] = p;
			weights[i][j] = 1.0;
		}
	}

	let knots_u = open_uniform_knots(n_u, deg_u);
	let knots_v = open_uniform_knots(n_v, deg_v);
	NurbsSurface::new(deg_u, deg_v, knots_u, knots_v, control, weights)
}

/// A parallel-transport (rotation-minimizing) frame swept along a polyline.
///
/// Returns, for each input point, a `(tangent, e1, e2)` orthonormal frame where
/// `tangent` follows the local path direction and `(e1, e2)` span the section
/// plane. The frame is initialised from [`perp_basis`] of the first tangent and
/// transported by the minimal rotation that maps each segment tangent to the
/// next, which avoids the twist a naive Frenet frame introduces at inflection
/// points. Degenerate (repeated) points reuse the previous tangent.
fn transport_frames(path: &[DVec3]) -> Vec<(DVec3, DVec3, DVec3)> {
	let n = path.len();
	if n == 0 {
		return Vec::new();
	}
	if n == 1 {
		let (e1, e2) = perp_basis(DVec3::Z);
		return vec![(DVec3::Z, e1, e2)];
	}

	// Per-point tangents: forward difference for the first point, backward for the
	// last, central for the interior; fall back to the previous valid tangent when
	// a segment collapses.
	let mut tangents: Vec<DVec3> = Vec::with_capacity(n);
	let mut last = (path[1] - path[0]).normalize_or_zero();
	if last == DVec3::ZERO {
		last = DVec3::Z;
	}
	for i in 0..n {
		let raw = if i == 0 {
			path[1] - path[0]
		} else if i == n - 1 {
			path[n - 1] - path[n - 2]
		} else {
			path[i + 1] - path[i - 1]
		};
		let t = raw.normalize_or_zero();
		if t != DVec3::ZERO {
			last = t;
		}
		tangents.push(last);
	}

	let mut frames: Vec<(DVec3, DVec3, DVec3)> = Vec::with_capacity(n);
	let (mut e1, mut e2) = perp_basis(tangents[0]);
	frames.push((tangents[0], e1, e2));
	for i in 1..n {
		let prev_t = tangents[i - 1];
		let cur_t = tangents[i];
		// Rotate the previous frame by the minimal rotation prev_t -> cur_t.
		let axis = prev_t.cross(cur_t);
		let sin = axis.length();
		let cos = prev_t.dot(cur_t).clamp(-1.0, 1.0);
		if sin > 1e-12 {
			let axis = axis / sin;
			let angle = sin.atan2(cos);
			e1 = rotate_about(e1, axis, angle);
		}
		// Re-orthonormalise against the new tangent to fight numeric drift, then
		// rebuild e2 as the right-handed completion of (tangent, e1).
		e1 = (e1 - cur_t * e1.dot(cur_t)).normalize_or_zero();
		if e1 == DVec3::ZERO {
			let (a, b) = perp_basis(cur_t);
			e1 = a;
			e2 = b;
		} else {
			e2 = cur_t.cross(e1);
		}
		frames.push((cur_t, e1, e2));
	}
	frames
}

/// Rodrigues rotation of `v` about unit `axis` by `angle` radians.
fn rotate_about(v: DVec3, axis: DVec3, angle: f64) -> DVec3 {
	let (s, c) = angle.sin_cos();
	v * c + axis.cross(v) * s + axis * (axis.dot(v) * (1.0 - c))
}

/// Sweep a profile polyline along a path polyline and skin a NURBS surface
/// through the swept copies.
///
/// The `profile` is expressed in its own local plane: its points are interpreted
/// in the path's first parallel-transport frame `(e1, e2)` so that the profile's
/// in-plane offset from its centroid is reproduced in the section plane at every
/// path point. As the path bends, a rotation-minimizing frame re-orients the
/// profile to stay perpendicular to the path tangent (`align` via
/// [`transport_frames`]), and the profile is translated to each path point.
///
/// The swept copies are then handed to [`loft`] with the given `degree` along the
/// path (`v`) direction, so the surface interpolates the end sections and blends
/// smoothly between them.
///
/// Returns `None` if the profile has fewer than two points, the path has fewer
/// than two points, or any input coordinate is non-finite.
pub fn sweep(profile: &[DVec3], path: &[DVec3], degree: usize) -> Option<NurbsSurface> {
	loft(&sweep_sections(profile, path)?, degree)
}

/// Sweep a **closed** `profile` loop along `path` into a closed B-rep [`Solid`].
///
/// The rotation-minimizing swept copies of the profile (see [`sweep`]) become the
/// section loops of [`loft_solid`], which adds end caps and stitches a watertight,
/// 2-manifold solid. The `profile` must be a closed loop of at least three points
/// wound counter-clockwise about its outward normal (as for [`loft_solid`]); the
/// `path` is treated as open (both ends are capped). Returns `None` on the same
/// degenerate inputs as [`sweep`] / [`loft_solid`].
pub fn sweep_solid(profile: &[DVec3], path: &[DVec3]) -> Option<Solid> {
	loft_solid(&sweep_sections(profile, path)?)
}

/// Build the rotation-minimizing swept copies of `profile` at every point of
/// `path` — the shared core of [`sweep`] and [`sweep_solid`].
///
/// The profile is expressed in the path's first parallel-transport frame, then
/// re-planted into each station's transported frame so it stays perpendicular to
/// the path tangent as the path bends. Returns `None` if either polyline has fewer
/// than two points or any coordinate is non-finite.
fn sweep_sections(profile: &[DVec3], path: &[DVec3]) -> Option<Vec<Vec<DVec3>>> {
	if profile.len() < 2 || path.len() < 2 {
		return None;
	}
	for p in profile.iter().chain(path.iter()) {
		if !p.is_finite() {
			return None;
		}
	}

	let frames = transport_frames(path);
	let base = frames[0];

	// Express the profile in the base frame as (a, b) local coordinates so it can
	// be re-planted into each transported frame. We project onto the base frame's
	// section plane; any component along the base tangent is preserved as an
	// along-path offset added in the local tangent of each station.
	let local: Vec<(f64, f64, f64)> = profile
		.iter()
		.map(|&p| {
			let d = p - path[0];
			(d.dot(base.1), d.dot(base.2), d.dot(base.0))
		})
		.collect();

	// Build one profile copy per path point in its transported frame.
	let mut sections: Vec<Vec<DVec3>> = Vec::with_capacity(path.len());
	for (k, &origin) in path.iter().enumerate() {
		let (t, e1, e2) = frames[k];
		let section: Vec<DVec3> = local.iter().map(|&(a, b, c)| origin + e1 * a + e2 * b + t * c).collect();
		sections.push(section);
	}

	Some(sections)
}

/// Centroid of a set of points.
fn centroid(points: &[DVec3]) -> DVec3 {
	points.iter().fold(DVec3::ZERO, |acc, &p| acc + p) / points.len().max(1) as f64
}

/// A planar triangle face whose outward normal follows the `a → b → c` winding.
fn tri_face(pos: &[DVec3], a: u32, b: u32, c: u32) -> FaceInput {
	let pa = pos[a as usize];
	let normal = (pos[b as usize] - pa).cross(pos[c as usize] - pa).normalize_or_zero();
	FaceInput { boundary: vec![a, b, c], surface: Surface::Plane { origin: pa, normal } }
}

/// Loft a **closed B-rep [`Solid`]** through a stack of closed section loops.
///
/// Each entry of `sections` is one closed boundary loop of `M` points (the loop
/// wraps `M-1 → 0`; do not duplicate the first point as the last). **All sections
/// must share the same point count `M`**, be ordered along the loft direction, and
/// wind consistently (counter-clockwise seen from the +direction side). Adjacent
/// sections are joined by triangulated lateral faces and the two end sections are
/// closed with centroid-fan caps, with every face oriented so its outward normal
/// points away from the body — so the result is a watertight, 2-manifold solid
/// (unlike [`loft`], which yields an open skinned surface).
///
/// Returns `None` for fewer than two sections, fewer than three points per
/// section, mismatched section lengths, or any non-finite coordinate.
pub fn loft_solid(sections: &[Vec<DVec3>]) -> Option<Solid> {
	let n = sections.len();
	if n < 2 {
		return None;
	}
	let m = sections[0].len();
	if m < 3 {
		return None;
	}
	for s in sections {
		if s.len() != m || s.iter().any(|p| !p.is_finite()) {
			return None;
		}
	}

	let mut pos: Vec<DVec3> = Vec::with_capacity(n * m + 2);
	for s in sections {
		pos.extend_from_slice(s);
	}
	let idx = |j: usize, i: usize| (j * m + i) as u32;

	let mut faces: Vec<FaceInput> = Vec::new();

	// Lateral band: each quad (section j..j+1, edge i..i+1, wrapping) split into two
	// outward triangles. Winding a→b→c, a→c→d keeps shared edges anti-parallel so
	// `from_faces` matches every twin.
	for j in 0..n - 1 {
		for i in 0..m {
			let i1 = (i + 1) % m;
			let (a, b, c, d) = (idx(j, i), idx(j, i1), idx(j + 1, i1), idx(j + 1, i));
			faces.push(tri_face(&pos, a, b, c));
			faces.push(tri_face(&pos, a, c, d));
		}
	}

	// Bottom cap (section 0): centroid fan wound i+1 → i so its normal faces away
	// from section 1 (opposite the loft direction) and its loop edges are the
	// reverse of the adjacent lateral edges.
	let bottom = pos.len() as u32;
	pos.push(centroid(&sections[0]));
	for i in 0..m {
		let i1 = (i + 1) % m;
		faces.push(tri_face(&pos, bottom, idx(0, i1), idx(0, i)));
	}

	// Top cap (section n-1): centroid fan wound i → i+1 so its normal faces along
	// the loft direction.
	let top = pos.len() as u32;
	pos.push(centroid(&sections[n - 1]));
	for i in 0..m {
		let i1 = (i + 1) % m;
		faces.push(tri_face(&pos, top, idx(n - 1, i), idx(n - 1, i1)));
	}

	Some(Solid::from_faces(pos, faces))
}

// ============================================================================
// Freeform booleans — the shipped slice (DESIGN_GUIDE §24 item 1)
// ============================================================================
//
// A freeform (NURBS) face can now be a boolean operand for ONE bounded, honest
// slice: a **planar half-space cut** (difference / intersection) of a
// single-patch [`FreeformSolid`]. The routing contract is *exact-surface,
// tolerance-curve*: the B-spline patch itself stays exact (every emitted trim
// point is an `S(u,v)` evaluation of the untouched rational surface), while the
// plane∩patch intersection curve is a polyline refined to a stated chord
// tolerance carried in the result ([`FreeformCut::chord_tol`]). The cut solid
// is a watertight mesh whose seam vertices are Newton-snapped onto the exact
// intersection curve; validity is gated here and the result withheld on any
// failure. Everything outside the slice refuses loudly through
// [`crate::checked::try_freeform_boolean`] with a message naming the slice.

/// A solid bounded by freeform (NURBS) patches carried as exact sidecars over a
/// watertight triangle mesh — the boolean-operand form of a freeform body.
///
/// `mesh` is the closed boundary tessellation (vertices in a patch's region lie
/// ON that patch, to the mesh's `f32` resolution); `faces` are the exact
/// rational patches with their trim rings, exactly as
/// [`crate::import_step_freeform`] returns them. The analytic
/// [`Surface`] enum has no freeform variant, so this pair
/// — not a [`Solid`] — is what a freeform boolean operates on.
#[derive(Clone, Debug)]
pub struct FreeformSolid {
	/// Closed boundary tessellation of the body.
	pub mesh: Mesh,
	/// Exact NURBS identity of the freeform region(s) of `mesh`.
	pub faces: Vec<FreeformFace>,
}

/// Build the canonical single-patch freeform test body: a **freeform plate** —
/// the solid bounded above by the exact patch `surf`, below by the plane
/// `z = base_z`, and laterally by ruled walls dropped from the patch boundary
/// straight down to the base.
///
/// Requirements (checked; `None` otherwise): `surf` must be a heightfield over
/// its base — every sampled patch point strictly above `base_z` — with a plan
/// (xy) boundary that projects to a simple polygon (the base cap is
/// ear-clipped). The mesh samples the patch on an `(nu+1)×(nv+1)` grid, so its
/// top vertices lie exactly on the patch (at `f32` storage resolution); the
/// returned [`FreeformFace`] ring is the patch boundary at the same grid
/// density, wound counter-clockwise in the chart.
pub fn freeform_plate(surf: &NurbsSurface, base_z: f64, nu: usize, nv: usize) -> Option<FreeformSolid> {
	let (nu, nv) = (nu.max(2), nv.max(2));
	let ((u0, u1), (v0, v1)) = surf.domain();
	let at = |i: usize, j: usize| surf.point_at(u0 + (u1 - u0) * i as f64 / nu as f64, v0 + (v1 - v0) * j as f64 / nv as f64);

	// Top grid, exactly on the patch.
	let mut grid: Vec<Vec<DVec3>> = Vec::with_capacity(nu + 1);
	for i in 0..=nu {
		let mut row = Vec::with_capacity(nv + 1);
		for j in 0..=nv {
			let p = at(i, j);
			if !p.is_finite() || p.z <= base_z + 1e-9 {
				return None; // not a heightfield above the base
			}
			row.push(p);
		}
		grid.push(row);
	}

	let mut mesh = Mesh::new();
	let idx = |i: usize, j: usize| (i * (nv + 1) + j) as u32;
	for row in &grid {
		for p in row {
			mesh.positions.push(p.as_vec3());
		}
	}
	// Top: two triangles per cell; the induced boundary direction is the
	// perimeter walk used below, so walls pair edge-for-edge.
	for i in 0..nu {
		for j in 0..nv {
			mesh.indices.extend_from_slice(&[idx(i, j), idx(i + 1, j), idx(i + 1, j + 1)]);
			mesh.indices.extend_from_slice(&[idx(i, j), idx(i + 1, j + 1), idx(i, j + 1)]);
		}
	}
	// Perimeter walk in the top triangulation's induced boundary direction,
	// carried as (i, j) so the mesh indices, the exact-f64 plan outline and the
	// patch trim ring below all come from ONE order.
	let mut ring_ij: Vec<(usize, usize)> = Vec::with_capacity(2 * (nu + nv));
	for i in 0..nu {
		ring_ij.push((i, 0));
	}
	for j in 0..nv {
		ring_ij.push((nu, j));
	}
	for i in (1..=nu).rev() {
		ring_ij.push((i, nv));
	}
	for j in (1..=nv).rev() {
		ring_ij.push((0, j));
	}
	let ring_idx: Vec<u32> = ring_ij.iter().map(|&(i, j)| idx(i, j)).collect();
	// Bottom ring: the projection of the top ring onto the base plane.
	let base_start = mesh.positions.len() as u32;
	let m = ring_idx.len();
	for &t in &ring_idx {
		let p = mesh.positions[t as usize];
		mesh.positions.push(kernel_core::math::Vec3::new(p.x, p.y, base_z as f32));
	}
	// Walls: each top boundary edge b_k→b_{k+1} is reused REVERSED.
	for k in 0..m {
		let kn = (k + 1) % m;
		let (bt0, bt1) = (ring_idx[k], ring_idx[kn]);
		let (bb0, bb1) = (base_start + k as u32, base_start + kn as u32);
		mesh.indices.extend_from_slice(&[bt1, bt0, bb0]);
		mesh.indices.extend_from_slice(&[bt1, bb0, bb1]);
	}
	// Base cap: ear-clip the projected boundary polygon walked in REVERSE, so
	// its induced edges pair with the walls' bottom edges. The outline comes
	// from the EXACT f64 patch samples, not the f32 mesh positions — a plan
	// outline's long straight runs are exactly the place where f32 wobble
	// masquerades as reflex geometry.
	let poly: Vec<DVec2> = ring_ij.iter().rev().map(|&(i, j)| DVec2::new(grid[i][j].x, grid[i][j].y)).collect();
	let tris = earclip(&poly)?;
	let rev_base = |local: usize| base_start + (m - 1 - local) as u32;
	for [a, b, c] in tris {
		mesh.indices.extend_from_slice(&[rev_base(a), rev_base(b), rev_base(c)]);
	}
	mesh.weld(1e-6);
	mesh.compute_normals();
	mesh.ensure_outward();
	if !mesh.is_watertight() {
		return None;
	}

	let ring: Vec<DVec3> = ring_ij.iter().map(|&(i, j)| grid[i][j]).collect();
	Some(FreeformSolid { mesh, faces: vec![FreeformFace { surface: surf.clone(), rings: vec![ring] }] })
}

/// One connected component of a plane ∩ B-spline-patch intersection, traced in
/// the patch's parameter chart. `uv` are normalized chart coordinates in
/// `[0,1]²`; `points[k]` is the exact surface evaluation `S(uv[k])` — every
/// point lies ON the exact patch by construction, and on the plane to the
/// measured `plane_dev`. Adjacent points chord-approximate the true curve to
/// the `chord_tol` passed to [`plane_patch_curves`].
#[derive(Clone, Debug)]
pub struct PatchPlaneCurve {
	/// Normalized chart coordinates of the polyline.
	pub uv: Vec<DVec2>,
	/// `S(uv)` — the polyline in model space, exactly on the patch.
	pub points: Vec<DVec3>,
	/// Whether the curve closes on itself (an island crossing, e.g. a plane
	/// clipping a dome) rather than running boundary-to-boundary.
	pub closed: bool,
	/// Measured max |signed plane distance| over the emitted points.
	pub plane_dev: f64,
}

/// Chart-space Newton: slide a normalized `uv` onto the zero set of
/// `F(u,v) = (S(u,v) − origin)·normal` (the plane∩patch curve), clamped to the
/// chart. Returns the refined point and its residual |F|.
fn newton_to_plane(surf: &NurbsSurface, origin: DVec3, normal: DVec3, mut uv: DVec2) -> (DVec2, f64) {
	let ((u0, u1), (v0, v1)) = surf.domain();
	let (span_u, span_v) = (u1 - u0, v1 - v0);
	let mut best = uv;
	let mut best_f = f64::INFINITY;
	for _ in 0..12 {
		let (u, v) = (u0 + span_u * uv.x, v0 + span_v * uv.y);
		let f = (surf.point_at(u, v) - origin).dot(normal);
		if f.abs() < best_f {
			best_f = f.abs();
			best = uv;
		}
		if f.abs() < 1e-13 * (1.0 + origin.length()) {
			break;
		}
		let (du, dv) = surf.partials(u, v);
		let g = DVec2::new(du.dot(normal) * span_u, dv.dot(normal) * span_v);
		let g2 = g.length_squared();
		if g2 < 1e-30 {
			break; // tangential: no transverse direction to slide along
		}
		uv = (uv - g * (f / g2)).clamp(DVec2::ZERO, DVec2::ONE);
	}
	(best, best_f)
}

/// Trace the intersection curve(s) of an (infinite) plane with the exact
/// B-spline patch, **in the patch's parameter chart** — the freeform half of
/// the SSI chart pattern. Marching squares on a `grid×grid` chart sampling
/// finds every transversal crossing at that resolution (features smaller than
/// one chart cell are below this function's resolution — raise `grid` for fine
/// geometry); each polyline vertex is then a chart point whose surface
/// evaluation lies exactly on the patch and on the plane to ~1e-12·scale, and
/// each polyline is adaptively midpoint-refined until the 3-D chord deviation
/// of every segment is ≤ `chord_tol` (model units) — the *stated* tolerance of
/// this exact-surface / tolerance-curve routing. Open curves start and end
/// exactly on the chart boundary; closed island crossings are flagged.
pub fn plane_patch_curves(
	surf: &NurbsSurface,
	plane_origin: DVec3,
	plane_normal: DVec3,
	chord_tol: f64,
	grid: usize,
) -> Vec<PatchPlaneCurve> {
	let n = grid.max(8);
	let normal = plane_normal.normalize_or_zero();
	if normal == DVec3::ZERO || chord_tol <= 0.0 {
		return Vec::new();
	}
	let ((u0, u1), (v0, v1)) = surf.domain();
	let (span_u, span_v) = (u1 - u0, v1 - v0);
	let value = |fu: f64, fv: f64| (surf.point_at(u0 + span_u * fu, v0 + span_v * fv) - plane_origin).dot(normal);
	let at3 = |uv: DVec2| surf.point_at(u0 + span_u * uv.x, v0 + span_v * uv.y);

	// Chart sample grid of F.
	let mut f = vec![vec![0.0_f64; n + 1]; n + 1]; // f[i][j] at (i/n, j/n)
	for (i, row) in f.iter_mut().enumerate() {
		for (j, cell) in row.iter_mut().enumerate() {
			*cell = value(i as f64 / n as f64, j as f64 / n as f64);
		}
	}
	let pos = |x: f64| x >= 0.0;

	// One root per sign-changing grid edge, found by bisection ALONG the edge
	// (the border coordinate stays exactly 0.0/1.0 on border edges).
	let bisect = |a: DVec2, b: DVec2, fa: f64, fb: f64| -> DVec2 {
		let (mut a, mut b, mut fa, _fb) = (a, b, fa, fb);
		for _ in 0..60 {
			let mid = (a + b) * 0.5;
			let fm = value(mid.x, mid.y);
			if fm.abs() < 1e-13 {
				return mid;
			}
			if pos(fm) == pos(fa) {
				a = mid;
				fa = fm;
			} else {
				b = mid;
			}
		}
		(a + b) * 0.5
	};
	// Edge keys: horizontal (0, i, j) = (i,j)→(i+1,j); vertical (1, i, j) = (i,j)→(i,j+1).
	let hkey = |i: usize, j: usize| i * (n + 1) + j;
	let vkey = |i: usize, j: usize| i * (n + 1) + j;
	let mut hroot: Vec<Option<DVec2>> = vec![None; (n + 1) * (n + 1)];
	let mut vroot: Vec<Option<DVec2>> = vec![None; (n + 1) * (n + 1)];
	for i in 0..n {
		for j in 0..=n {
			let (fa, fb) = (f[i][j], f[i + 1][j]);
			if pos(fa) != pos(fb) {
				let a = DVec2::new(i as f64 / n as f64, j as f64 / n as f64);
				let b = DVec2::new((i + 1) as f64 / n as f64, j as f64 / n as f64);
				hroot[hkey(i, j)] = Some(bisect(a, b, fa, fb));
			}
		}
	}
	for i in 0..=n {
		for j in 0..n {
			let (fa, fb) = (f[i][j], f[i][j + 1]);
			if pos(fa) != pos(fb) {
				let a = DVec2::new(i as f64 / n as f64, j as f64 / n as f64);
				let b = DVec2::new(i as f64 / n as f64, (j + 1) as f64 / n as f64);
				vroot[vkey(i, j)] = Some(bisect(a, b, fa, fb));
			}
		}
	}

	// Per-cell segments between edge roots (marching squares with the centre
	// sign as the saddle decider). Node ids: horizontal roots then vertical.
	let hid = |i: usize, j: usize| hkey(i, j);
	let vid = |i: usize, j: usize| (n + 1) * (n + 1) + vkey(i, j);
	let mut adj: std::collections::HashMap<usize, Vec<usize>> = std::collections::HashMap::new();
	let mut link = |a: usize, b: usize| {
		adj.entry(a).or_default().push(b);
		adj.entry(b).or_default().push(a);
	};
	for i in 0..n {
		for j in 0..n {
			let mut ends: Vec<usize> = Vec::with_capacity(4);
			if hroot[hkey(i, j)].is_some() {
				ends.push(hid(i, j)); // bottom
			}
			if hroot[hkey(i, j + 1)].is_some() {
				ends.push(hid(i, j + 1)); // top
			}
			if vroot[vkey(i, j)].is_some() {
				ends.push(vid(i, j)); // left
			}
			if vroot[vkey(i + 1, j)].is_some() {
				ends.push(vid(i + 1, j)); // right
			}
			match ends.len() {
				2 => link(ends[0], ends[1]),
				4 => {
					// Saddle: pair crossings around the corners whose sign differs
					// from the centre.
					let centre = pos(value((i as f64 + 0.5) / n as f64, (j as f64 + 0.5) / n as f64));
					if centre == pos(f[i][j]) {
						// isolated corners are (i+1,j) and (i,j+1)
						link(hid(i, j), vid(i + 1, j)); // around corner (i+1, j)
						link(hid(i, j + 1), vid(i, j)); // around corner (i, j+1)
					} else {
						// isolated corners are (i,j) and (i+1,j+1)
						link(hid(i, j), vid(i, j)); // around corner (i, j)
						link(hid(i, j + 1), vid(i + 1, j)); // around corner (i+1, j+1)
					}
				}
				_ => {}
			}
		}
	}

	let root_uv = |id: usize| -> DVec2 {
		if id < (n + 1) * (n + 1) {
			hroot[id].expect("linked horizontal root")
		} else {
			vroot[id - (n + 1) * (n + 1)].expect("linked vertical root")
		}
	};
	let on_border = |uv: DVec2| uv.x == 0.0 || uv.x == 1.0 || uv.y == 0.0 || uv.y == 1.0;

	// Chain walk: open chains first (from degree-1 nodes, deterministic id
	// order), then leftover closed loops.
	let mut visited: std::collections::HashSet<usize> = std::collections::HashSet::new();
	let mut chains: Vec<(Vec<usize>, bool)> = Vec::new();
	let mut node_ids: Vec<usize> = adj.keys().copied().collect();
	node_ids.sort_unstable();
	for &start in &node_ids {
		if visited.contains(&start) || adj[&start].len() != 1 {
			continue;
		}
		let mut chain = vec![start];
		visited.insert(start);
		let mut cur = start;
		loop {
			let next = adj[&cur].iter().copied().find(|nb| !visited.contains(nb));
			match next {
				Some(nb) => {
					visited.insert(nb);
					chain.push(nb);
					cur = nb;
				}
				None => break,
			}
		}
		chains.push((chain, false));
	}
	for &start in &node_ids {
		if visited.contains(&start) {
			continue;
		}
		let mut chain = vec![start];
		visited.insert(start);
		let mut cur = start;
		loop {
			let next = adj[&cur].iter().copied().find(|nb| !visited.contains(nb));
			match next {
				Some(nb) => {
					visited.insert(nb);
					chain.push(nb);
					cur = nb;
				}
				None => break,
			}
		}
		chains.push((chain, true));
	}

	// Adaptive chord refinement + receipts.
	let mut out = Vec::new();
	for (chain, closed) in chains {
		if chain.len() < 2 {
			continue;
		}
		let base: Vec<DVec2> = chain.iter().map(|&id| root_uv(id)).collect();
		let mut uv: Vec<DVec2> = vec![base[0]];
		let seg_count = if closed { base.len() } else { base.len() - 1 };
		for s in 0..seg_count {
			let (a, b) = (base[s], base[(s + 1) % base.len()]);
			// Iterative midpoint refinement of segment a→b to the chord tolerance.
			let mut stack = vec![(a, b, 0usize)];
			let mut emitted: Vec<DVec2> = Vec::new();
			while let Some((pa, pb, depth)) = stack.pop() {
				let (m, _res) = newton_to_plane(surf, plane_origin, normal, (pa + pb) * 0.5);
				let (qa, qb, qm) = (at3(pa), at3(pb), at3(m));
				let chord = qb - qa;
				let t = if chord.length_squared() > 1e-30 { ((qm - qa).dot(chord) / chord.length_squared()).clamp(0.0, 1.0) } else { 0.5 };
				let dev = (qm - (qa + chord * t)).length();
				if dev <= chord_tol || depth >= 12 {
					emitted.push(pb);
				} else {
					stack.push((m, pb, depth + 1));
					stack.push((pa, m, depth + 1));
				}
			}
			uv.extend(emitted);
		}
		if closed {
			uv.pop(); // the wrap point duplicates uv[0]
		}
		if uv.len() < 2 {
			continue;
		}
		let points: Vec<DVec3> = uv.iter().map(|&q| at3(q)).collect();
		let plane_dev = points.iter().fold(0.0_f64, |m, p| m.max(((*p - plane_origin).dot(normal)).abs()));
		// Border endpoints of an open chain must sit exactly on the chart border
		// (bisection along a border edge keeps the border coordinate exact).
		debug_assert!(closed || (on_border(uv[0]) && on_border(*uv.last().unwrap())), "open chain must end on the chart border");
		out.push(PatchPlaneCurve { uv, points, closed, plane_dev });
	}
	out
}

/// Ear-clip triangulation of a simple polygon (2-D, either winding). Returns
/// local-index triangles wound WITH the input order, or `None` when the polygon
/// is degenerate (near-zero area, or no ear found — e.g. self-intersecting).
///
/// The load-bearing contract (a cap built from these triangles is only
/// watertight if it holds): **every directed polygon edge `P_k → P_{k+1}`
/// appears exactly once, forward, across the output triangles.** So a
/// collinear vertex is never *dropped* — that would silently swallow two
/// boundary edges into one chord — it is only ever consumed by clipping a
/// strictly-convex ear elsewhere, which is always available on a simple
/// polygon of positive area. Long straight runs (the norm on cut boundaries
/// and faceted plan outlines) therefore triangulate into real, non-degenerate
/// triangles rather than needles.
///
/// Tolerance, stated honestly: both the ear (convexity) and the containment
/// predicate use one signed-area epsilon `1e-7 ×` the polygon's own bounding
/// box diagonal², which is *deliberately* far above `f64` noise. Cut
/// boundaries arrive from an `f32` mesh, where a straight run's vertices wobble
/// off the line by ~1e-6 × their magnitude; a tighter epsilon reads that wobble
/// as genuine reflex/containment and blocks every ear (measured: an 80-vertex
/// plan outline of a 40 mm plate failed outright). The cost of the loose
/// epsilon is that a *genuinely* reflex vertex shallower than that band is
/// treated as collinear — a sub-noise distinction on this input class. A
/// polygon that cannot be clipped returns `None`, which every caller turns
/// into a loud refusal, never a guessed cap.
fn earclip(poly: &[DVec2]) -> Option<Vec<[usize; 3]>> {
	let n = poly.len();
	if n < 3 {
		return None;
	}
	let mut area2 = 0.0;
	let (mut lo, mut hi) = (poly[0], poly[0]);
	for i in 0..n {
		let (a, b) = (poly[i], poly[(i + 1) % n]);
		area2 += a.x * b.y - b.x * a.y;
		lo = lo.min(b);
		hi = hi.max(b);
	}
	let eps = 1e-7 * (hi - lo).length_squared().max(1e-30);
	if area2.abs() * 0.5 <= eps {
		return None;
	}
	// Work CCW; if the input is CW, clip the reversed order and map back (the
	// emitted triangles then follow the ORIGINAL winding either way).
	let ccw = area2 > 0.0;
	let map = |k: usize| if ccw { k } else { n - 1 - k };
	let mut idx: Vec<usize> = (0..n).collect(); // indices into the CCW view
	let p = |k: usize| poly[map(k)];
	let cross = |o: DVec2, a: DVec2, b: DVec2| (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
	let mut tris: Vec<[usize; 3]> = Vec::with_capacity(n - 2);
	let mut guard = 0usize;
	while idx.len() > 3 {
		let m = idx.len();
		let mut clipped = false;
		for k in 0..m {
			let (ia, ib, ic) = (idx[(k + m - 1) % m], idx[k], idx[(k + 1) % m]);
			let (a, b, c) = (p(ia), p(ib), p(ic));
			// Signed-area epsilon scaled to THIS ear, not to the whole polygon: a
			// global epsilon is simultaneously too tight for a big coarse ear and
			// far too loose for a small dense one (measured: a 247-vertex cut loop
			// blocked every candidate under a polygon-wide epsilon).
			let tri_eps = eps.min(1e-6 * (b - a).length_squared().max((c - b).length_squared()).max((a - c).length_squared()));
			let conv = cross(a, b, c);
			if conv <= tri_eps {
				// Reflex OR collinear: not a clippable ear. A collinear vertex is
				// deliberately left in place (see the contract above) — clipping a
				// strictly-convex ear elsewhere will consume it correctly.
				continue;
			}
			// A genuine ear: no other polygon vertex inside the CLOSED triangle.
			// "Closed" is load-bearing on collinear runs: a candidate ear whose
			// chord `a→c` lies ALONG a straight boundary run has every vertex of
			// that run exactly on its edge, and a strict inside-test waves them
			// through — the ear then swallows the whole run into one chord,
			// silently orphaning its boundary edges (measured: a 40 mm plate's
			// 80-vertex plan outline triangulated to a pinched, zero-area
			// remainder that consumed the left edge). Testing the closed
			// triangle blocks exactly that.
			let mut blocked = false;
			for &io in &idx {
				if io == ia || io == ib || io == ic {
					continue;
				}
				let q = p(io);
				if cross(a, b, q) >= -tri_eps && cross(b, c, q) >= -tri_eps && cross(c, a, q) >= -tri_eps {
					blocked = true;
					break;
				}
			}
			if !blocked {
				tris.push([ia, ib, ic]);
				idx.remove(k);
				clipped = true;
				break;
			}
		}
		if !clipped {
			return None; // stuck: not a simple polygon
		}
		guard += 1;
		if guard > 4 * n {
			return None;
		}
	}
	tris.push([idx[0], idx[1], idx[2]]);
	// Map from the CCW view back to input-local indices. For CW input the view
	// reverses edge directions, so the mapped triangle must be flipped — the
	// contract either way: every directed polygon edge `P_k → P_{k+1}` appears
	// exactly once, FORWARD, among the output triangles.
	Some(tris.into_iter().map(|[a, b, c]| if ccw { [a, b, c] } else { [map(c), map(b), map(a)] }).collect())
}

/// Why a freeform boolean was refused or withheld. Everything the shipped
/// slice does not cover arrives as [`OutOfScope`](FreeformBoolError::OutOfScope)
/// with a message NAMING the slice; a geometrically degenerate in-slice cut
/// (island crossing, grazing contact, full removal) arrives as
/// [`DegenerateCut`](FreeformBoolError::DegenerateCut); and an in-slice cut
/// whose result fails the watertightness gate is WITHHELD as
/// [`NotWatertight`](FreeformBoolError::NotWatertight) — an invalid freeform
/// boolean never propagates silently, mirroring the [`crate::try_difference`]
/// contract.
#[derive(Clone, Debug)]
pub enum FreeformBoolError {
	/// The operand/op pair is outside the shipped slice (the message names it).
	OutOfScope {
		/// What was asked for.
		detail: String,
		/// The chord tolerance the slice would have used (model units).
		chord_tol: f64,
	},
	/// The cut is in-slice but geometrically degenerate; nothing was built.
	DegenerateCut {
		/// What the tracer/splitter found.
		detail: String,
	},
	/// The cut was computed but its mesh failed the validity gate; withheld.
	NotWatertight {
		/// Unpaired directed edges of the withheld mesh.
		boundary_edges: usize,
		/// Undirected edges not shared by exactly two triangles.
		non_manifold_edges: usize,
	},
}

impl std::fmt::Display for FreeformBoolError {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		match self {
			FreeformBoolError::OutOfScope { detail, chord_tol } => write!(
				f,
				"freeform boolean support: planar half-space cuts (difference/intersection) of a single-patch freeform solid only, exact-surface / tolerance-curve routing (chord tol {chord_tol}); {detail} is out of scope"
			),
			FreeformBoolError::DegenerateCut { detail } => {
				write!(f, "freeform planar cut is degenerate: {detail}; nothing was built")
			}
			FreeformBoolError::NotWatertight { boundary_edges, non_manifold_edges } => write!(
				f,
				"freeform planar cut produced a non-watertight mesh ({boundary_edges} boundary edges, {non_manifold_edges} non-manifold edges); result withheld"
			),
		}
	}
}

impl std::error::Error for FreeformBoolError {}

/// Tuning for [`freeform_plane_cut`]. `Default` resolves every field
/// automatically from the operand's scale.
#[derive(Clone, Copy, Debug)]
pub struct FreeformCutOptions {
	/// Chord tolerance (model units) of the intersection polyline — the stated
	/// tolerance of the exact-surface / tolerance-curve contract. `0` = auto:
	/// `1e-4 ×` the patch control-net diagonal.
	pub chord_tol: f64,
	/// Chart sampling density of the marching-squares tracer (cells per axis).
	pub grid: usize,
	/// Snap band for pulling seam vertices onto the exact curve. `0` = auto:
	/// [`crate::auto_seam_band`] of the operand mesh.
	pub seam_band: f64,
}

impl Default for FreeformCutOptions {
	fn default() -> Self {
		Self { chord_tol: 0.0, grid: 64, seam_band: 0.0 }
	}
}

/// A completed freeform planar cut. The routing contract, stated exactly:
/// the B-spline patch is EXACT (untouched; both trimmed halves reference it
/// verbatim, and every trim-ring point is an `S(u,v)` evaluation of it); the
/// intersection **curve** is a polyline refined to `chord_tol`; the cut
/// **solid** is a watertight mesh whose patch-region seam vertices are
/// Newton-snapped onto the exact curve, with the cut cross-section capped by
/// planar ear-clipped facets (the solid's seam follows the operand's
/// tessellation density — the `curve` field, not the mesh, carries the
/// chord-tol-refined polyline).
#[derive(Clone, Debug)]
pub struct FreeformCut {
	/// The kept side of the cut: closed, gated watertight before return.
	pub mesh: Mesh,
	/// The kept trimmed half of the patch (`None` when the patch was entirely
	/// on the removed side).
	pub kept_face: Option<FreeformFace>,
	/// The removed trimmed half (`None` when the patch was untouched).
	pub dropped_face: Option<FreeformFace>,
	/// The plane∩patch intersection polyline (empty when the plane misses the
	/// patch), refined to `chord_tol`; every point lies exactly on the patch.
	pub curve: Vec<DVec3>,
	/// Chart coordinates of `curve` (normalized `[0,1]²`).
	pub curve_uv: Vec<DVec2>,
	/// The chord tolerance the curve was refined to (resolved value).
	pub chord_tol: f64,
	/// Measured max |plane distance| over the curve points.
	pub curve_plane_dev: f64,
	/// Total area of the planar cap facets added on the cut plane.
	pub cap_area: f64,
}

/// The exact patch point of a normalized chart coordinate.
fn chart_point(surf: &NurbsSurface, uv: DVec2) -> DVec3 {
	let ((u0, u1), (v0, v1)) = surf.domain();
	surf.point_at(u0 + (u1 - u0) * uv.x, v0 + (v1 - v0) * uv.y)
}

/// Plane field `F` of a normalized chart coordinate.
fn chart_plane_value(surf: &NurbsSurface, origin: DVec3, normal: DVec3, uv: DVec2) -> f64 {
	(chart_point(surf, uv) - origin).dot(normal)
}

/// Cyclic border parameter `t ∈ [0,4)` of a chart point lying ON the border
/// (side 0: `v=0`, `u` ascending; 1: `u=1`, `v` ascending; 2: `v=1`, `u`
/// descending; 3: `u=0`, `v` descending — the CCW walk). `None` off-border.
fn border_t(uv: DVec2) -> Option<f64> {
	if uv.y == 0.0 && uv.x < 1.0 {
		Some(uv.x)
	} else if uv.x == 1.0 && uv.y < 1.0 {
		Some(1.0 + uv.y)
	} else if uv.y == 1.0 && uv.x > 0.0 {
		Some(2.0 + (1.0 - uv.x))
	} else if uv.x == 0.0 && uv.y > 0.0 {
		Some(3.0 + (1.0 - uv.y))
	} else {
		None
	}
}

/// The chart point of a cyclic border parameter (inverse of [`border_t`]).
fn border_uv(t: f64) -> DVec2 {
	let t = t.rem_euclid(4.0);
	let side = t.floor().min(3.0);
	let frac = t - side;
	match side as usize {
		0 => DVec2::new(frac, 0.0),
		1 => DVec2::new(1.0, frac),
		2 => DVec2::new(1.0 - frac, 1.0),
		_ => DVec2::new(0.0, 1.0 - frac),
	}
}

/// Border samples strictly between the cyclic parameters `ta → tb` (walking
/// CCW, i.e. ascending `t` mod 4), at `1/grid` spacing — corners included
/// (they sit on the sample lattice). Endpoints themselves are NOT emitted.
fn border_arc_samples(ta: f64, tb: f64, grid: usize) -> Vec<DVec2> {
	let g = grid.max(4) as f64;
	let len = (tb - ta).rem_euclid(4.0);
	let mut out = Vec::new();
	let mut k = (ta * g).floor() + 1.0;
	loop {
		let t = k / g;
		if t - ta >= len {
			break; // delta grows monotonically, so this always terminates
		}
		out.push(border_uv(t));
		k += 1.0;
	}
	out
}

/// Split the full-domain chart rectangle by ONE open transversal curve into
/// the kept and dropped trim rings (normalized chart coordinates, CCW). The
/// kept side is where `F·keep_sign ≥ 0`.
fn split_chart_rings(
	surf: &NurbsSurface,
	curve: &PatchPlaneCurve,
	plane_origin: DVec3,
	normal: DVec3,
	keep_sign: f64,
	grid: usize,
) -> Result<(Vec<DVec2>, Vec<DVec2>), FreeformBoolError> {
	let e_start = curve.uv[0];
	let e_end = *curve.uv.last().expect("curve has points");
	if (e_start - e_end).length() < 1e-9 {
		return Err(FreeformBoolError::DegenerateCut {
			detail: "the crossing curve enters and leaves the patch boundary at the same point (tangential graze)".into(),
		});
	}
	let (t0, t1) = match (border_t(e_start), border_t(e_end)) {
		(Some(a), Some(b)) => (a, b),
		_ => {
			return Err(FreeformBoolError::DegenerateCut {
				detail: "an open crossing curve did not terminate on the patch boundary".into(),
			})
		}
	};
	// Arc A walks CCW from the curve's END back to its START; arc B is the
	// complement (START → END). Classify each by its cyclic midpoint sample.
	let scale = 1.0 + plane_origin.length() + surf.control.iter().flatten().fold(0.0_f64, |m, p| m.max(p.length()));
	let mid_of = |ta: f64, tb: f64| border_uv(ta + 0.5 * (tb - ta).rem_euclid(4.0));
	let f_a = chart_plane_value(surf, plane_origin, normal, mid_of(t1, t0));
	let f_b = chart_plane_value(surf, plane_origin, normal, mid_of(t0, t1));
	if f_a.abs() < 1e-9 * scale || f_b.abs() < 1e-9 * scale || (f_a > 0.0) == (f_b > 0.0) {
		return Err(FreeformBoolError::DegenerateCut {
			detail: format!(
				"side classification of the split is ambiguous (boundary-arc plane fields {f_a:.3e} / {f_b:.3e}) — the cut grazes the patch boundary"
			),
		});
	}
	// Ring with arc A: END → (border CCW) → START → (curve forward) → END.
	let ring_with_arc_a: Vec<DVec2> =
		std::iter::once(e_end).chain(border_arc_samples(t1, t0, grid)).chain(curve.uv[..curve.uv.len() - 1].iter().copied()).collect();
	// Ring with arc B: START → (border CCW) → END → (curve backward) → START.
	let ring_with_arc_b: Vec<DVec2> =
		std::iter::once(e_start).chain(border_arc_samples(t0, t1, grid)).chain(curve.uv[1..].iter().rev().copied()).collect();
	if f_a * keep_sign > 0.0 {
		Ok((ring_with_arc_a, ring_with_arc_b))
	} else {
		Ok((ring_with_arc_b, ring_with_arc_a))
	}
}

/// Snap the open-boundary (seam) vertices of a trimmed mesh onto the exact
/// plane∩patch curve: a boundary vertex that projects onto the patch within
/// `4·band` is slid along the patch to the plane (chart Newton) and moved —
/// unless the motion would exceed `10·band` (a safety leash; such a vertex is
/// left where the trim put it). Interior vertices are never touched. Returns
/// the worst residual |plane distance| among the moved vertices.
fn snap_boundary_to_curve(mesh: &mut Mesh, surf: &NurbsSurface, origin: DVec3, normal: DVec3, band: f64) -> f64 {
	let seeds = surf.projection_seeds(24);
	let mut worst = 0.0_f64;
	for loop_v in boundary_loops(mesh) {
		for &vi in &loop_v {
			let p = mesh.positions[vi as usize].as_dvec3();
			if (p - origin).dot(normal).abs() > 4.0 * band {
				continue; // not a cut-plane vertex
			}
			let tol = (4.0 * band) / (1.0 + p.length());
			let Some(uv) = surf.project(&seeds, p, tol) else {
				continue; // not on the patch: a wall/base seam vertex
			};
			let (uv2, res) = newton_to_plane(surf, origin, normal, uv);
			let q = chart_point(surf, uv2);
			if (q - p).length() <= 10.0 * band {
				mesh.positions[vi as usize] = q.as_vec3();
				worst = worst.max(res);
			}
		}
	}
	worst
}

/// Cut a single-patch [`FreeformSolid`] by an (infinite) plane and keep one
/// side — **the shipped freeform-boolean slice** (difference with the removed
/// half-space, or intersection with the kept one; see
/// [`crate::checked::try_freeform_boolean`] for the operand-level dispatch and
/// refusal boundary).
///
/// Routing contract, stated exactly (see [`FreeformCut`]): exact surface,
/// tolerance curve. The patch is never approximated — the intersection curve
/// is traced in its parameter chart and refined until every chord deviates
/// ≤ `chord_tol` (resolved value returned in the result); the trimmed halves
/// reference the untouched surface with rings evaluated on it; the cut solid
/// is the operand tessellation trimmed against the plane, its patch seam
/// snapped onto the exact curve and the cross-section capped with planar
/// facets — gated watertight before return, withheld otherwise.
///
/// Refusals are loud and specific: out-of-slice operands (multi-patch bodies,
/// hole-ringed or partial trims), degenerate cuts (island/multi-crossings at
/// chart resolution, grazes, full removal), and any validity-gate failure.
pub fn freeform_plane_cut(
	solid: &FreeformSolid,
	plane_origin: DVec3,
	plane_normal: DVec3,
	keep: Keep,
	opts: &FreeformCutOptions,
) -> Result<FreeformCut, FreeformBoolError> {
	// ---- resolve scales & slice checks --------------------------------------
	if solid.mesh.is_empty() {
		return Err(FreeformBoolError::DegenerateCut { detail: "the operand mesh is empty".into() });
	}
	let bb = solid.mesh.aabb();
	let diag = (bb.max - bb.min).as_dvec3().length().max(1e-9);
	let chord_tol = if opts.chord_tol > 0.0 { opts.chord_tol } else { diag * 1e-4 };
	let normal = plane_normal.normalize_or_zero();
	if normal == DVec3::ZERO || !plane_origin.is_finite() {
		return Err(FreeformBoolError::DegenerateCut { detail: "cut plane has a zero or non-finite normal/origin".into() });
	}
	if solid.faces.len() != 1 {
		return Err(FreeformBoolError::OutOfScope {
			detail: format!("an operand carrying {} freeform patches (the slice takes exactly one)", solid.faces.len()),
			chord_tol,
		});
	}
	let face = &solid.faces[0];
	if face.rings.len() != 1 {
		return Err(FreeformBoolError::OutOfScope {
			detail: format!("a patch trimmed with {} inner ring(s) (the slice takes full-domain trims only)", face.rings.len() - 1),
			chord_tol,
		});
	}
	let surf = &face.surface;
	// NaN weights fail `is_finite`, so this catches the whole unusable set
	// (non-positive, infinite, NaN) without a negated float comparison.
	if surf.weights.iter().flatten().any(|w| w <= &0.0 || !w.is_finite()) {
		return Err(FreeformBoolError::OutOfScope { detail: "a patch with non-positive or non-finite weights".into(), chord_tol });
	}
	// The slice treats the trim as the full parameter rectangle — verify the
	// recorded ring actually hugs the domain boundary.
	{
		let seeds = surf.projection_seeds(16);
		let ring = &face.rings[0];
		let stride = (ring.len() / 32).max(1);
		for p in ring.iter().step_by(stride) {
			let ok = surf.project(&seeds, *p, 1e-4).map(|uv| uv.x.min(1.0 - uv.x).min(uv.y).min(1.0 - uv.y) <= 0.05).unwrap_or(false);
			if !ok {
				return Err(FreeformBoolError::OutOfScope {
					detail: "a patch trimmed inside its surface domain (the slice takes full-domain trims only)".into(),
					chord_tol,
				});
			}
		}
	}
	if !solid.mesh.is_watertight() {
		return Err(FreeformBoolError::DegenerateCut { detail: "the operand mesh is not watertight".into() });
	}

	// ---- trace the exact intersection curve in the chart --------------------
	let curves = plane_patch_curves(surf, plane_origin, normal, chord_tol, opts.grid);
	let keep_sign = if keep == Keep::Outside { 1.0 } else { -1.0 };
	let n_open = curves.iter().filter(|c| !c.closed).count();
	let n_closed = curves.len() - n_open;
	enum PatchCase {
		Untouched(bool), // true = patch on the kept side
		Split(PatchPlaneCurve),
	}
	let case = match (n_open, n_closed) {
		(0, 0) => {
			// No crossing at chart resolution. The control net decides exactly
			// where it can (hull property, positive weights); otherwise chart
			// samples decide, refusing on ambiguity.
			let scale = 1.0 + plane_origin.length();
			let ds: Vec<f64> = surf.control.iter().flatten().map(|p| (*p - plane_origin).dot(normal)).collect();
			let side = if ds.iter().all(|&d| d > 1e-12 * scale) {
				1.0
			} else if ds.iter().all(|&d| d < -1e-12 * scale) {
				-1.0
			} else {
				let samples = [
					DVec2::new(0.0, 0.0),
					DVec2::new(1.0, 0.0),
					DVec2::new(1.0, 1.0),
					DVec2::new(0.0, 1.0),
					DVec2::new(0.5, 0.5),
				];
				let fs: Vec<f64> = samples.iter().map(|&q| chart_plane_value(surf, plane_origin, normal, q)).collect();
				if fs.iter().all(|&x| x > 1e-9 * scale) {
					1.0
				} else if fs.iter().all(|&x| x < -1e-9 * scale) {
					-1.0
				} else {
					return Err(FreeformBoolError::DegenerateCut {
						detail: "the plane grazes the patch below the chart resolution — no crossing traced yet the patch is not decisively one-sided".into(),
					});
				}
			};
			PatchCase::Untouched(side * keep_sign > 0.0)
		}
		(1, 0) => PatchCase::Split(curves.into_iter().next().expect("one open curve")),
		_ => {
			return Err(FreeformBoolError::DegenerateCut {
				detail: format!(
					"the plane crosses the patch in {n_open} boundary-to-boundary and {n_closed} closed curve(s); the shipped slice splits exactly one boundary-to-boundary crossing (island and multi-crossing cuts are not yet split)"
				),
			})
		}
	};

	// ---- cut the mesh: trim → snap seam to the exact curve → cap ------------
	let plane_surface = Surface::Plane { origin: plane_origin, normal };
	let mut mesh = trim_mesh_by_surface(&solid.mesh, &plane_surface, keep);
	if mesh.is_empty() {
		return Err(FreeformBoolError::DegenerateCut { detail: "the cut removes the entire solid".into() });
	}
	let band = if opts.seam_band > 0.0 { opts.seam_band } else { crate::mesh_boolean::auto_seam_band(&solid.mesh) }.max(1e-9);
	snap_boundary_to_curve(&mut mesh, surf, plane_origin, normal, band);
	mesh.weld(1e-6);
	let loops = boundary_loops(&mesh);
	let (e1, e2) = perp_basis(normal);
	let mut cap_area = 0.0_f64;
	for loop_v in &loops {
		let poly: Vec<DVec2> = loop_v
			.iter()
			.map(|&i| {
				let d = mesh.positions[i as usize].as_dvec3() - plane_origin;
				DVec2::new(d.dot(e1), d.dot(e2))
			})
			.collect();
		let tris = earclip(&poly).ok_or_else(|| FreeformBoolError::DegenerateCut {
			detail: format!("the planar cap failed to triangulate (a cut loop of {} vertices is not a simple polygon)", poly.len()),
		})?;
		for [a, b, c] in tris {
			// The cap reuses the open boundary's REVERSED directed edges.
			mesh.push_triangle(loop_v[c], loop_v[b], loop_v[a]);
			let (pa, pb, pc) = (poly[a], poly[b], poly[c]);
			cap_area += 0.5 * ((pb.x - pa.x) * (pc.y - pa.y) - (pc.x - pa.x) * (pb.y - pa.y)).abs();
		}
	}
	mesh.compute_normals();
	mesh.ensure_outward();
	if !mesh.is_watertight() {
		return Err(FreeformBoolError::NotWatertight {
			boundary_edges: mesh.boundary_edge_count(),
			non_manifold_edges: mesh.non_manifold_edge_count(),
		});
	}

	// ---- emit the trimmed halves (exact surface, rings evaluated on it) -----
	let (kept_face, dropped_face, curve, curve_uv, curve_plane_dev) = match case {
		PatchCase::Untouched(kept) => {
			let kf = kept.then(|| face.clone());
			let df = (!kept).then(|| face.clone());
			(kf, df, Vec::new(), Vec::new(), 0.0)
		}
		PatchCase::Split(c) => {
			let (ring_kept, ring_dropped) = split_chart_rings(surf, &c, plane_origin, normal, keep_sign, opts.grid)?;
			let to3 = |ring: &[DVec2]| -> Vec<DVec3> { ring.iter().map(|&q| chart_point(surf, q)).collect() };
			let kf = FreeformFace { surface: surf.clone(), rings: vec![to3(&ring_kept)] };
			let df = FreeformFace { surface: surf.clone(), rings: vec![to3(&ring_dropped)] };
			(Some(kf), Some(df), c.points, c.uv, c.plane_dev)
		}
	};

	Ok(FreeformCut { mesh, kept_face, dropped_face, curve, curve_uv, chord_tol, curve_plane_dev, cap_area })
}

#[cfg(test)]
mod tests {
	use super::*;
	use kernel_core::math::DVec3;

	/// Sample a circle of `radius` centred at `center` in the `z = center.z`
	/// plane into `n` polyline points (open, not duplicating the seam).
	fn circle_polyline(center: DVec3, radius: f64, n: usize) -> Vec<DVec3> {
		(0..n)
			.map(|i| {
				let a = std::f64::consts::TAU * (i as f64) / (n as f64);
				center + DVec3::new(radius * a.cos(), radius * a.sin(), 0.0)
			})
			.collect()
	}

	#[test]
	fn loft_two_circles_is_a_tube_on_the_radius() {
		// Two identical circles a distance apart -> a (control-polygon) tube. Every
		// surface point must sit at ~the circle radius from the central axis (the
		// z-axis), and span the z-gap between the two rings.
		let r = 2.0;
		let n = 24;
		let bottom = circle_polyline(DVec3::new(0.0, 0.0, 0.0), r, n);
		let top = circle_polyline(DVec3::new(0.0, 0.0, 5.0), r, n);
		let surf = loft(&[bottom, top], 1).expect("loft of two circles");

		let mesh = surf.tessellate(32, 4);
		assert!(!mesh.is_empty(), "tube tessellation was empty");

		// The control points lie ON the radius-r circle, so by the convex-hull
		// property every B-spline surface point is at radial distance <= r. The
		// cubic u-blend rounds the open control polygon slightly inward, but with a
		// fine sampling it stays a tube wall well off the axis. Check the radial
		// distance stays in a band (clearly away from 0, never outside the circle)
		// for every vertex, and z stays inside the ring span.
		//
		// Lower bound: a degree-3 B-spline over an n-gon control polygon stays above
		// 0.9*r for n this dense; that proves "on the wall", not collapsed.
		let lo_band = r * 0.9;
		for p in &mesh.positions {
			let pd = p.as_dvec3();
			let radial = (pd.x * pd.x + pd.y * pd.y).sqrt();
			assert!(radial >= lo_band && radial <= r + 1e-9, "point off the tube wall: radial {radial} not in [{lo_band}, {r}]");
			assert!(pd.z >= -1e-3 && pd.z <= 5.0 + 1e-3, "z {} outside ring span", pd.z);
			assert!(pd.is_finite(), "non-finite surface point {pd:?}");
		}

		// A point evaluated at mid-v, mid-u must also be on the wall, proving the
		// v-blend (not just the control vertices) lands on the tube.
		let ((u_lo, u_hi), (v_lo, v_hi)) = surf.domain();
		let mid = surf.point_at((u_lo + u_hi) * 0.5, (v_lo + v_hi) * 0.5);
		let rmid = (mid.x * mid.x + mid.y * mid.y).sqrt();
		assert!(rmid >= lo_band && rmid <= r + 1e-9, "mid surface point off wall: {rmid}");
		assert!((mid.z - 2.5).abs() < 1.0, "mid z {} not near the ring midpoint", mid.z);
	}

	#[test]
	fn loft_to_collapsed_top_is_a_cone() {
		// Loft a circle to a degenerate top profile (all points coincident) ->
		// a cone-ish surface. It must be a valid, non-empty surface that tapers:
		// the apex row collapses to a point.
		let r = 3.0;
		let n = 16;
		let base = circle_polyline(DVec3::new(0.0, 0.0, 0.0), r, n);
		let apex = vec![DVec3::new(0.0, 0.0, 6.0); n];
		let surf = loft(&[base, apex], 1).expect("cone loft");

		let mesh = surf.tessellate(24, 6);
		assert!(!mesh.is_empty(), "cone tessellation empty");
		for p in &mesh.positions {
			assert!(p.as_dvec3().is_finite(), "non-finite cone point");
		}

		// Near the base (v_lo) the radius is large; near the apex (v_hi) it shrinks
		// toward zero. Sample a fixed u across the two v-extremes.
		let ((u_lo, u_hi), (v_lo, v_hi)) = surf.domain();
		let u = (u_lo + u_hi) * 0.5;
		let p_base = surf.point_at(u, v_lo);
		let p_apex = surf.point_at(u, v_hi);
		let rad = |p: DVec3| (p.x * p.x + p.y * p.y).sqrt();
		assert!(rad(p_base) > rad(p_apex) + 1e-6, "cone did not taper toward the apex");
		assert!(rad(p_apex) < 1e-9, "apex row did not collapse to the axis: {}", rad(p_apex));
	}

	#[test]
	fn sweep_square_along_straight_path_is_a_prism() {
		// A square profile swept along a straight vertical path -> a prism-like
		// surface. Tessellation must be non-empty and finite, and the swept cross
		// sections must preserve the square's in-plane size at every height.
		let half = 1.0;
		let profile =
			vec![DVec3::new(-half, -half, 0.0), DVec3::new(half, -half, 0.0), DVec3::new(half, half, 0.0), DVec3::new(-half, half, 0.0)];
		let path = vec![DVec3::new(0.0, 0.0, 0.0), DVec3::new(0.0, 0.0, 2.0), DVec3::new(0.0, 0.0, 4.0)];
		let surf = sweep(&profile, &path, 2).expect("square sweep");

		let mesh = surf.tessellate(8, 8);
		assert!(!mesh.is_empty(), "prism tessellation empty");

		let bb = mesh.aabb();
		// The straight path is along +Z and the transport frame keeps the profile in
		// the XY plane. By the convex-hull property the swept surface stays inside
		// the square's hull, so |x|,|y| <= half everywhere, and it spans the full
		// path height in Z. (A degree-3 B-spline over the 4 open corners interpolates
		// only the end corners, so the swept wall need not reach every corner — that
		// is correct general behaviour, not a prism with sharp faces.)
		assert!(
			bb.min.x >= -(half as f32) - 1e-3
				&& bb.max.x <= half as f32 + 1e-3
				&& bb.min.y >= -(half as f32) - 1e-3
				&& bb.max.y <= half as f32 + 1e-3,
			"profile escaped its convex hull: {bb:?}"
		);
		assert!(bb.min.z <= 1e-3 && bb.max.z >= 4.0 - 1e-3, "z extent off: {bb:?}");
		// The section must have real extent (not collapsed to a point/line).
		assert!((bb.max.x - bb.min.x) > half as f32 && (bb.max.y - bb.min.y) > half as f32, "swept section degenerate: {bb:?}");
		for p in &mesh.positions {
			assert!(p.as_dvec3().is_finite(), "non-finite prism point");
		}

		// Iso-(u_lo, v_lo) must reproduce the first control corner of the square
		// exactly (clamped knots in both directions interpolate the ends), proving
		// the basis partitions to a single control point at the clamped corner.
		let ((u_lo, _u_hi), (v_lo, _v_hi)) = surf.domain();
		let corner = surf.point_at(u_lo, v_lo);
		assert!((corner - profile[0]).length() < 1e-9, "start corner not interpolated: {corner:?}");
	}

	#[test]
	fn loft_solid_through_two_circles_is_a_closed_prism() {
		// Two identical m-gon rings a height apart loft into a CLOSED, manifold m-gon
		// prism. Its volume is exactly the m-gon cap area × height (straight lateral
		// walls between equal-radius rings), so check against the closed form.
		let (r, m, h) = (2.0, 24usize, 5.0);
		let bottom = circle_polyline(DVec3::new(0.0, 0.0, 0.0), r, m);
		let top = circle_polyline(DVec3::new(0.0, 0.0, h), r, m);
		let solid = crate::freeform::loft_solid(&[bottom, top]).expect("loft solid");

		let v = crate::validate::validate(&solid);
		let mgon_area = 0.5 * m as f64 * r * r * (std::f64::consts::TAU / m as f64).sin();
		let vol = crate::validate::volume(&solid);
		// vol == m-gon area × height geometrically; they agree only to FP summation noise.
		assert!(
			v.closed && v.manifold && (vol - mgon_area * h).abs() < 1e-4,
			"loft solid should be a closed manifold prism of volume area×height: closed={} manifold={} vol={vol} want {}",
			v.closed,
			v.manifold,
			mgon_area * h
		);
	}

	#[test]
	fn loft_solid_between_unequal_rings_is_a_closed_frustum() {
		// Different-radius rings loft into a closed manifold frustum (a truncated
		// cone). Exact volume is awkward for the triangulated walls, so check it is a
		// watertight solid of positive, plausibly-bounded volume.
		let m = 20usize;
		let big = circle_polyline(DVec3::new(0.0, 0.0, 0.0), 3.0, m);
		let small = circle_polyline(DVec3::new(0.0, 0.0, 4.0), 1.5, m);
		let solid = crate::freeform::loft_solid(&[big, small]).expect("frustum loft");

		let v = crate::validate::validate(&solid);
		let vol = crate::validate::volume(&solid);
		// Bounded by the two extreme cylinders' volumes (π·1.5²·4 ≈ 28.3 .. π·3²·4 ≈ 113).
		assert!(
			v.closed && v.manifold && vol > 25.0 && vol < 113.0,
			"frustum should be a closed manifold of plausible volume, got closed={} manifold={} vol={vol}",
			v.closed,
			v.manifold
		);
	}

	#[test]
	fn loft_solid_of_a_planar_frustum_has_exact_volume() {
		// A loft between two SQUARE sections has planar (trapezoidal) walls, so its volume is the
		// EXACT pyramidal-frustum value — not a faceted approximation (unlike the circular-ring
		// loft, whose curved walls are triangulated and only plausibly-bounded). A 4×4 base → 2×2
		// top over height 6: V = (h/3)(A_b + A_t + √(A_b·A_t)) = 2·(16 + 4 + 8) = 56.
		let sq = |h: f64, z: f64| vec![DVec3::new(-h, -h, z), DVec3::new(h, -h, z), DVec3::new(h, h, z), DVec3::new(-h, h, z)];
		let solid = crate::freeform::loft_solid(&[sq(2.0, 0.0), sq(1.0, 6.0)]).expect("planar frustum loft");
		let v = crate::validate::validate(&solid);
		assert!(
			v.closed && v.manifold && v.genus == 0 && (crate::validate::volume(&solid).abs() - 56.0).abs() < 1e-6,
			"planar square frustum must be a closed manifold of EXACT volume 56: {v:?} vol={}",
			crate::validate::volume(&solid).abs()
		);
	}

	#[test]
	fn sweep_solid_square_along_straight_path_is_a_closed_box() {
		// A closed square loop swept up a straight path is a closed manifold box; with
		// the path along +Z the cross-section is preserved, so the volume is side²×length.
		let half = 1.0;
		let profile =
			vec![DVec3::new(-half, -half, 0.0), DVec3::new(half, -half, 0.0), DVec3::new(half, half, 0.0), DVec3::new(-half, half, 0.0)];
		let path = vec![DVec3::new(0.0, 0.0, 0.0), DVec3::new(0.0, 0.0, 2.0), DVec3::new(0.0, 0.0, 4.0)];
		let solid = crate::freeform::sweep_solid(&profile, &path).expect("square sweep solid");

		let v = crate::validate::validate(&solid);
		let vol = crate::validate::volume(&solid);
		assert!(
			v.closed && v.manifold && (vol - 16.0).abs() < 1e-4,
			"swept square should be a closed manifold box of volume 16: closed={} manifold={} vol={vol}",
			v.closed,
			v.manifold
		);
	}

	#[test]
	fn sweep_solid_open_profile_returns_none() {
		// A 2-point "profile" is not a closed loop, so no solid can be formed.
		let path = vec![DVec3::ZERO, DVec3::Z];
		assert!(
			crate::freeform::sweep_solid(&[DVec3::new(-1.0, 0.0, 0.0), DVec3::new(1.0, 0.0, 0.0)], &path).is_none(),
			"a two-point profile cannot sweep into a solid"
		);
	}

	#[test]
	fn degenerate_inputs_return_none() {
		// Guard the documented degenerate cases rather than panicking.
		assert!(crate::freeform::loft_solid(&[]).is_none(), "empty section list");
		assert!(crate::freeform::loft_solid(&[circle_polyline(DVec3::ZERO, 1.0, 8)]).is_none(), "single section");
		assert!(
			crate::freeform::loft_solid(&[vec![DVec3::ZERO, DVec3::X], vec![DVec3::ZERO, DVec3::X]]).is_none(),
			"sections with fewer than three points"
		);
		assert!(loft(&[], 2).is_none(), "empty profile list");
		assert!(loft(&[vec![DVec3::ZERO, DVec3::X]], 2).is_none(), "single profile");
		assert!(loft(&[vec![DVec3::ZERO, DVec3::X], vec![DVec3::Y]], 2).is_none(), "mismatched profile lengths");
		assert!(sweep(&[DVec3::ZERO], &[DVec3::ZERO, DVec3::Z], 2).is_none(), "too-short profile");
		assert!(sweep(&[DVec3::ZERO, DVec3::X], &[DVec3::ZERO], 2).is_none(), "too-short path");
	}

	#[test]
	fn earclip_traverses_polygon_edges_forward_in_both_windings() {
		// The cap stitching depends on ONE contract: every directed polygon edge
		// P_k→P_{k+1} appears exactly once, forward, among the output triangles
		// (so emitting the triangles reversed supplies exactly the reversed
		// boundary edges a watertight cap needs). Check it on a non-convex L,
		// in both windings, plus the total-area invariant.
		let l_ccw = vec![
			DVec2::new(0.0, 0.0),
			DVec2::new(4.0, 0.0),
			DVec2::new(4.0, 1.0),
			DVec2::new(1.0, 1.0),
			DVec2::new(1.0, 3.0),
			DVec2::new(0.0, 3.0),
		];
		let l_cw: Vec<DVec2> = l_ccw.iter().rev().copied().collect();
		// A 4×3 rectangle walked CCW with every side split into unit steps —
		// long COLLINEAR runs. This is the case that matters in practice (plan
		// outlines and cut boundaries are full of them) and the one that
		// regressed: an earlier draft *dropped* collinear vertices, silently
		// swallowing two boundary edges into one chord and cracking every cap.
		let mut subdivided = Vec::new();
		for i in 0..4 {
			subdivided.push(DVec2::new(i as f64, 0.0));
		}
		for j in 0..3 {
			subdivided.push(DVec2::new(4.0, j as f64));
		}
		for i in (1..=4).rev() {
			subdivided.push(DVec2::new(i as f64, 3.0));
		}
		for j in (1..=3).rev() {
			subdivided.push(DVec2::new(0.0, j as f64));
		}
		for (label, poly, want_area) in [("L ccw", &l_ccw, 6.0), ("L cw", &l_cw, 6.0), ("collinear-run rectangle", &subdivided, 12.0)] {
			let tris = earclip(poly).unwrap_or_else(|| panic!("{label}: earclip failed on a simple polygon"));
			let n = poly.len();
			let mut area = 0.0;
			let mut edge_used = std::collections::HashSet::new();
			for [a, b, c] in &tris {
				let (pa, pb, pc) = (poly[*a], poly[*b], poly[*c]);
				area += 0.5 * ((pb.x - pa.x) * (pc.y - pa.y) - (pc.x - pa.x) * (pb.y - pa.y)).abs();
				for (s, t) in [(*a, *b), (*b, *c), (*c, *a)] {
					if (s + 1) % n == t {
						assert!(edge_used.insert(s), "{label}: polygon edge {s}->{t} used twice");
					}
					assert!(
						(t + 1) % n != s,
						"{label}: triangle traverses polygon edge {t}->{s} BACKWARD — cap stitching would double an edge"
					);
				}
			}
			assert_eq!(
				edge_used.len(),
				n,
				"{label}: every one of the {n} polygon edges must appear exactly once forward (got {})",
				edge_used.len()
			);
			assert!((area - want_area).abs() < 1e-12, "{label}: triangulated area {area} != polygon area {want_area}");
		}
	}

	#[test]
	fn sweep_along_bent_path_stays_finite_and_follows_path() {
		// A profile swept along an L-shaped path must produce a finite, non-empty
		// surface whose stations track the path turn (rotation-minimizing frame).
		let profile = vec![DVec3::new(-0.5, 0.0, 0.0), DVec3::new(0.5, 0.0, 0.0), DVec3::new(0.0, 0.5, 0.0)];
		let path = vec![DVec3::new(0.0, 0.0, 0.0), DVec3::new(0.0, 0.0, 3.0), DVec3::new(3.0, 0.0, 3.0)];
		let surf = sweep(&profile, &path, 2).expect("bent sweep");
		let mesh = surf.tessellate(6, 12);
		assert!(!mesh.is_empty(), "bent sweep empty");
		for p in &mesh.positions {
			assert!(p.as_dvec3().is_finite(), "non-finite bent-sweep point");
		}
		// The end section should sit near the path's final point (the profile is
		// small relative to the 3-unit legs).
		let ((u_lo, u_hi), (_v_lo, v_hi)) = surf.domain();
		let end = surf.point_at((u_lo + u_hi) * 0.5, v_hi);
		assert!((end - DVec3::new(3.0, 0.0, 3.0)).length() < 1.0, "end station off path tip: {end:?}");
	}
}
