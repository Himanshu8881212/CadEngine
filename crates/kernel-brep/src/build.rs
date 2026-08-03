// Copyright (c) LMCAD. Licensed under the MIT License.

//! Solid constructors — the MVP authoring surface (no booleans needed).
//!
//! Primitives (box, cylinder, sphere, cone), linear sweep ([`extrude`]), and
//! rotational sweep ([`revolve`]). Every face is emitted with an outward-facing
//! winding via [`orient`], which is exactly the invariant the half-edge twin
//! matcher needs, so the result is a consistently oriented closed manifold.

use std::f64::consts::TAU;

use kernel_core::math::{DVec2, DVec3};

use crate::geom::{perp_basis, Curve, Surface};
use crate::topo::{FaceInput, FaceLoops, Solid, VertexId};

/// Newell's method: area-weighted normal of a (possibly non-planar) polygon,
/// following its winding (CCW ⇒ right-hand normal).
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

/// Clean a closed 2D profile: drop consecutive coincident points and a closing
/// duplicate. Returns `None` if the result is degenerate (fewer than 3 distinct
/// points, or effectively zero area) — such a profile has no valid solid, so the
/// constructor returns an empty [`Solid`] rather than building non-manifold
/// topology (which would trip the half-edge twin assertion).
fn sanitize_profile(profile: &[DVec2]) -> Option<Vec<DVec2>> {
	let mut pts: Vec<DVec2> = Vec::with_capacity(profile.len());
	for &p in profile {
		if pts.last().is_none_or(|&q| (p - q).length_squared() > 1e-18) {
			pts.push(p);
		}
	}
	if pts.len() >= 2 && (pts[0] - *pts.last().unwrap()).length_squared() <= 1e-18 {
		pts.pop();
	}
	if pts.len() < 3 {
		return None;
	}
	let mut area2 = 0.0;
	for i in 0..pts.len() {
		let j = (i + 1) % pts.len();
		area2 += pts[i].x * pts[j].y - pts[j].x * pts[i].y;
	}
	if area2.abs() * 0.5 < 1e-9 {
		return None;
	}
	Some(pts)
}

/// Return `boundary` wound so its polygon normal agrees with `outward`.
fn orient(boundary: Vec<u32>, positions: &[DVec3], outward: DVec3) -> Vec<u32> {
	let poly: Vec<DVec3> = boundary.iter().map(|&i| positions[i as usize]).collect();
	if newell_normal(&poly).dot(outward) < 0.0 {
		let mut b = boundary;
		b.reverse();
		b
	} else {
		boundary
	}
}

/// An axis-aligned box from opposite corners.
pub fn cuboid(min: DVec3, max: DVec3) -> Solid {
	let (lo, hi) = (min.min(max), min.max(max));
	let positions = vec![
		DVec3::new(lo.x, lo.y, lo.z),
		DVec3::new(hi.x, lo.y, lo.z),
		DVec3::new(lo.x, hi.y, lo.z),
		DVec3::new(hi.x, hi.y, lo.z),
		DVec3::new(lo.x, lo.y, hi.z),
		DVec3::new(hi.x, lo.y, hi.z),
		DVec3::new(lo.x, hi.y, hi.z),
		DVec3::new(hi.x, hi.y, hi.z),
	];
	let face = |quad: [u32; 4], origin: DVec3, normal: DVec3| FaceInput {
		boundary: orient(quad.to_vec(), &positions, normal),
		surface: Surface::Plane { origin, normal },
	};
	let faces = vec![
		face([0, 1, 3, 2], DVec3::new(0.0, 0.0, lo.z), -DVec3::Z),
		face([4, 5, 7, 6], DVec3::new(0.0, 0.0, hi.z), DVec3::Z),
		face([0, 1, 5, 4], DVec3::new(0.0, lo.y, 0.0), -DVec3::Y),
		face([2, 3, 7, 6], DVec3::new(0.0, hi.y, 0.0), DVec3::Y),
		face([0, 2, 6, 4], DVec3::new(lo.x, 0.0, 0.0), -DVec3::X),
		face([1, 3, 7, 5], DVec3::new(hi.x, 0.0, 0.0), DVec3::X),
	];
	// Canonical face order: 0=-Z 1=+Z 2=-Y 3=+Y 4=-X 5=+X. Naming the faces makes the
	// box's edges nameable (and stable across a resize), so a fillet can target them.
	Solid::from_faces(positions, faces).with_primitive_names()
}

/// A capped cylinder: `base` disk centre, unit `axis`, `radius`, `height`,
/// faceted into `segments` angular sectors.
pub fn cylinder(base: DVec3, axis: DVec3, radius: f64, height: f64, segments: usize) -> Solid {
	if !height.is_finite() || height == 0.0 || !radius.is_finite() || radius <= 0.0 {
		return Solid::default();
	}
	let segments = segments.max(3);
	let axis = axis.normalize();
	let (e1, e2) = perp_basis(axis);
	let top = base + axis * height;
	// Cap normals follow the height sign (a negative height puts `top` below `base`).
	let (cap_lo, cap_hi) = (-axis * height.signum(), axis * height.signum());

	let mut pos = Vec::with_capacity(segments * 2);
	for k in 0..segments {
		let a = TAU * k as f64 / segments as f64;
		pos.push(base + (e1 * a.cos() + e2 * a.sin()) * radius);
	}
	for k in 0..segments {
		let a = TAU * k as f64 / segments as f64;
		pos.push(top + (e1 * a.cos() + e2 * a.sin()) * radius);
	}

	let s = segments as u32;
	let mut faces = Vec::new();
	// Caps (single n-gon each).
	faces.push(FaceInput {
		boundary: orient((0..s).collect(), &pos, cap_lo),
		surface: Surface::Plane { origin: base, normal: cap_lo },
	});
	faces.push(FaceInput {
		boundary: orient((s..2 * s).collect(), &pos, cap_hi),
		surface: Surface::Plane { origin: top, normal: cap_hi },
	});
	// Sides.
	let cyl = Surface::Cylinder { origin: base, axis, radius };
	for k in 0..segments {
		let k1 = (k + 1) % segments;
		let mid = TAU * (k as f64 + 0.5) / segments as f64;
		let radial = e1 * mid.cos() + e2 * mid.sin();
		let quad = vec![k as u32, k1 as u32, s + k1 as u32, s + k as u32];
		faces.push(FaceInput { boundary: orient(quad, &pos, radial), surface: cyl });
	}
	// Tag the two circular rim edge-loops with their exact analytic circle (rim
	// vertices 0..s and s..2s are evenly spaced on the base / top circles), and name
	// the faces so the rims are referenceable.
	let mut solid = Solid::from_faces(pos, faces).with_primitive_names();
	let base_circle = Curve::Circle { center: base, normal: axis, radius };
	let top_circle = Curve::Circle { center: top, normal: axis, radius };
	for k in 0..segments {
		let k1 = ((k + 1) % segments) as u32;
		solid.set_edge_curve(VertexId(k as u32), VertexId(k1), base_circle);
		solid.set_edge_curve(VertexId(s + k as u32), VertexId(s + k1), top_circle);
	}
	solid
}

/// A cone with `base` disk centre, unit `axis`, `base_radius`, `height` to the
/// apex, faceted into `segments` sectors.
pub fn cone(base: DVec3, axis: DVec3, base_radius: f64, height: f64, segments: usize) -> Solid {
	if !height.is_finite() || height == 0.0 || !base_radius.is_finite() || base_radius <= 0.0 {
		return Solid::default();
	}
	let segments = segments.max(3);
	let axis = axis.normalize();
	let (e1, e2) = perp_basis(axis);
	let apex = base + axis * height;
	// Base-cap normal follows the height sign (a negative height puts the apex below).
	let cap_lo = -axis * height.signum();

	let mut pos = Vec::with_capacity(segments + 1);
	for k in 0..segments {
		let a = TAU * k as f64 / segments as f64;
		pos.push(base + (e1 * a.cos() + e2 * a.sin()) * base_radius);
	}
	let apex_id = segments as u32;
	pos.push(apex);

	let s = segments as u32;
	let mut faces = Vec::new();
	faces.push(FaceInput {
		boundary: orient((0..s).collect(), &pos, cap_lo),
		surface: Surface::Plane { origin: base, normal: cap_lo },
	});
	// `axis` points base→apex; the cone surface opens from the apex toward the
	// base, so its surface axis is -axis.
	// Surface axis must point apex→base and the half-angle stay positive regardless of
	// the height sign, or `Surface::project` collapses every lateral point onto the apex.
	let surf = Surface::Cone { apex, axis: -axis * height.signum(), half_angle: (base_radius / height.abs()).atan() };
	for k in 0..segments {
		let k1 = (k + 1) % segments;
		let mid = TAU * (k as f64 + 0.5) / segments as f64;
		let radial = e1 * mid.cos() + e2 * mid.sin();
		let tri = vec![k as u32, k1 as u32, apex_id];
		faces.push(FaceInput { boundary: orient(tri, &pos, radial), surface: surf });
	}
	// Tag the base rim with its exact analytic circle (vertices 0..segments).
	let mut solid = Solid::from_faces(pos, faces).with_primitive_names();
	let base_circle = Curve::Circle { center: base, normal: axis, radius: base_radius };
	for k in 0..segments {
		let k1 = ((k + 1) % segments) as u32;
		solid.set_edge_curve(VertexId(k as u32), VertexId(k1), base_circle);
	}
	solid
}

/// A UV sphere with `u_segments` longitudes and `v_segments` latitudes.
pub fn sphere(center: DVec3, radius: f64, u_segments: usize, v_segments: usize) -> Solid {
	let u = u_segments.max(3);
	let v = v_segments.max(2);
	let surf = Surface::Sphere { center, radius };

	let mut pos = Vec::new();
	pos.push(center + DVec3::Z * radius); // north pole = index 0
	for r in 1..v {
		let theta = std::f64::consts::PI * r as f64 / v as f64;
		for c in 0..u {
			let phi = TAU * c as f64 / u as f64;
			pos.push(
				center + DVec3::new(theta.sin() * phi.cos(), theta.sin() * phi.sin(), theta.cos()) * radius,
			);
		}
	}
	let south = pos.len() as u32;
	pos.push(center - DVec3::Z * radius);

	// Ring `r` (1-based) starts at this vertex index.
	let ring = |r: usize, c: usize| 1 + (r - 1) * u + (c % u);
	let mut faces = Vec::new();
	let hint = |ids: &[u32]| {
		let cen: DVec3 = ids.iter().map(|&i| pos[i as usize]).sum::<DVec3>() / ids.len() as f64;
		cen - center
	};
	// Top fan.
	for c in 0..u {
		let tri = vec![0u32, ring(1, c) as u32, ring(1, c + 1) as u32];
		faces.push(FaceInput { boundary: orient(tri.clone(), &pos, hint(&tri)), surface: surf });
	}
	// Middle quads.
	for r in 1..v - 1 {
		for c in 0..u {
			let quad = vec![
				ring(r, c) as u32,
				ring(r, c + 1) as u32,
				ring(r + 1, c + 1) as u32,
				ring(r + 1, c) as u32,
			];
			faces.push(FaceInput { boundary: orient(quad.clone(), &pos, hint(&quad)), surface: surf });
		}
	}
	// Bottom fan.
	for c in 0..u {
		let tri = vec![south, ring(v - 1, c + 1) as u32, ring(v - 1, c) as u32];
		faces.push(FaceInput { boundary: orient(tri.clone(), &pos, hint(&tri)), surface: surf });
	}
	Solid::from_faces(pos, faces)
}

/// Linear extrusion of a closed CCW 2D `profile` (in the XY plane) by `height`
/// along +Z. Profile edges sweep planar side faces.
///
/// Degenerate input (fewer than 3 distinct points, or a zero-area profile) yields
/// an empty [`Solid`] rather than panicking.
pub fn extrude(profile: &[DVec2], height: f64) -> Solid {
	// A zero / non-finite height has no volume; reject it (the cap-orientation sign
	// below assumes a definite direction).
	if !height.is_finite() || height == 0.0 {
		return Solid::default();
	}
	let profile = match sanitize_profile(profile) {
		Some(p) => p,
		None => return Solid::default(),
	};
	let n = profile.len();
	let mut pos = Vec::with_capacity(2 * n);
	for p in &profile {
		pos.push(DVec3::new(p.x, p.y, 0.0));
	}
	for p in &profile {
		pos.push(DVec3::new(p.x, p.y, height));
	}

	let nn = n as u32;
	// Cap outward normals must track the extrusion direction: for a negative height
	// the "top" loop is physically below the base, so the base cap faces +Z and the
	// top cap faces −Z. (The side faces self-orient from geometry already.)
	let sgn = height.signum();
	let (cap_lo, cap_hi) = (-DVec3::Z * sgn, DVec3::Z * sgn);
	let mut faces = Vec::new();
	faces.push(FaceInput {
		boundary: orient((0..nn).collect(), &pos, cap_lo),
		surface: Surface::Plane { origin: DVec3::ZERO, normal: cap_lo },
	});
	faces.push(FaceInput {
		boundary: orient((nn..2 * nn).collect(), &pos, cap_hi),
		surface: Surface::Plane { origin: DVec3::new(0.0, 0.0, height), normal: cap_hi },
	});
	for k in 0..n {
		let k1 = (k + 1) % n;
		let dir = profile[k1] - profile[k];
		let outward = DVec3::new(dir.y, -dir.x, 0.0).normalize_or_zero(); // right of travel (CCW ⇒ outward)
		let origin = DVec3::new(profile[k].x, profile[k].y, 0.0);
		let quad = vec![k as u32, k1 as u32, nn + k1 as u32, nn + k as u32];
		faces.push(FaceInput {
			boundary: orient(quad, &pos, outward),
			surface: Surface::Plane { origin, normal: outward },
		});
	}
	// Faces: 0=bottom cap, 1=top cap, then one side per profile edge in order. Naming
	// them makes the prism's edges nameable (and stable across a parametric edit).
	Solid::from_faces(pos, faces).with_primitive_names()
}

/// Tapered (**drafted**) extrusion: like [`extrude`], but every wall slopes inward
/// by `draft` radians from vertical, so the top cap is the profile inset by
/// `height · tan(draft)` (a mitered offset — each edge moves inward along its own
/// normal and adjacent offsets meet at the bisector). This is the **draft / taper**
/// that every moulded or cast part needs to release from its tool. A positive
/// `draft` narrows toward the top; `draft == 0` is an ordinary prism. The walls and
/// both caps are planar, so the solid is exact and its volume matches the prismatoid
/// (truncated-pyramid) closed form.
///
/// Convex profiles only: a concave vertex can self-intersect under inset. Degenerate
/// input — bad height/draft, a spike vertex, or a draft so steep the top collapses —
/// yields an empty [`Solid`] rather than a self-intersecting one.
pub fn extrude_tapered(profile: &[DVec2], height: f64, draft: f64) -> Solid {
	if !height.is_finite() || height == 0.0 || !draft.is_finite() {
		return Solid::default();
	}
	if draft == 0.0 {
		return extrude(profile, height);
	}
	// The miter formula below assumes counter-clockwise winding (outward normal is
	// to the right of travel), so force it.
	let profile = match sanitize_profile(profile) {
		Some(p) => to_ccw(p),
		None => return Solid::default(),
	};
	let n = profile.len();
	let d = height * draft.tan();
	// Per-edge outward unit normals (right of CCW travel).
	let outward2d: Vec<DVec2> = (0..n)
		.map(|k| {
			let dir = profile[(k + 1) % n] - profile[k];
			DVec2::new(dir.y, -dir.x).normalize_or_zero()
		})
		.collect();
	// Mitered top vertices: inset each vertex by `d` along the bisector of its two
	// adjacent edge normals — `v − d·(nₚ+nₙ)/(1+nₚ·nₙ)` — so every wall makes the
	// same angle with vertical.
	let mut top = Vec::with_capacity(n);
	for i in 0..n {
		let n_next = outward2d[i];
		let n_prev = outward2d[(i + n - 1) % n];
		let denom = 1.0 + n_prev.dot(n_next);
		if denom.abs() < 1e-9 {
			return Solid::default(); // near-degenerate spike vertex
		}
		top.push(profile[i] - d * (n_prev + n_next) / denom);
	}
	// Reject a collapsed / flipped top loop (draft too steep for this height).
	if signed_area_2d(&top) <= 1e-12 {
		return Solid::default();
	}

	let mut pos = Vec::with_capacity(2 * n);
	for p in &profile {
		pos.push(DVec3::new(p.x, p.y, 0.0));
	}
	for p in &top {
		pos.push(DVec3::new(p.x, p.y, height));
	}

	let nn = n as u32;
	let sgn = height.signum();
	let (cap_lo, cap_hi) = (-DVec3::Z * sgn, DVec3::Z * sgn);
	let mut faces = Vec::new();
	faces.push(FaceInput {
		boundary: orient((0..nn).collect(), &pos, cap_lo),
		surface: Surface::Plane { origin: DVec3::ZERO, normal: cap_lo },
	});
	faces.push(FaceInput {
		boundary: orient((nn..2 * nn).collect(), &pos, cap_hi),
		surface: Surface::Plane { origin: DVec3::new(0.0, 0.0, height), normal: cap_hi },
	});
	for k in 0..n {
		let k1 = (k + 1) % n;
		let b0 = pos[k];
		let b1 = pos[k1];
		let t0 = pos[nn as usize + k];
		// Each wall is the planar trapezoid b0→b1→t1→t0; its normal tilts with the
		// draft. Orient it outward using the horizontal outward direction as the hint.
		let h_out = DVec3::new(outward2d[k].x, outward2d[k].y, 0.0);
		let mut normal = (b1 - b0).cross(t0 - b0).normalize_or_zero();
		if normal.dot(h_out) < 0.0 {
			normal = -normal;
		}
		let quad = vec![k as u32, k1 as u32, nn + k1 as u32, nn + k as u32];
		faces.push(FaceInput {
			boundary: orient(quad, &pos, normal),
			surface: Surface::Plane { origin: b0, normal },
		});
	}
	Solid::from_faces(pos, faces).with_primitive_names()
}

/// Signed area of a 2D polygon (positive ⇒ counter-clockwise).
fn signed_area_2d(p: &[DVec2]) -> f64 {
	let n = p.len();
	let mut a = 0.0;
	for i in 0..n {
		let q = p[(i + 1) % n];
		a += p[i].x * q.y - q.x * p[i].y;
	}
	0.5 * a
}

/// Force a loop counter-clockwise (reverse it if it is clockwise).
fn to_ccw(mut p: Vec<DVec2>) -> Vec<DVec2> {
	if signed_area_2d(&p) < 0.0 {
		p.reverse();
	}
	p
}

/// Linear extrusion of a closed `outer` 2D profile **with `holes`** along +Z by
/// `height` — a washer / annulus / any plate-with-through-holes. The outer loop and
/// every hole are each forced counter-clockwise; the top and bottom are emitted as
/// multi-loop faces (outer + hole rings) and the holes get their own inward-facing
/// lateral walls, so the result is a single closed manifold solid of genus = number
/// of holes. With no holes this is just [`extrude`]. Degenerate input (bad height,
/// < 3-point outer) yields an empty [`Solid`].
pub fn extrude_with_holes(outer: &[DVec2], holes: &[Vec<DVec2>], height: f64) -> Solid {
	if !height.is_finite() || height == 0.0 {
		return Solid::default();
	}
	let outer = match sanitize_profile(outer) {
		Some(p) => to_ccw(p),
		None => return Solid::default(),
	};
	let holes: Vec<Vec<DVec2>> = holes.iter().filter_map(|h| sanitize_profile(h).map(to_ccw)).collect();
	if holes.is_empty() {
		return extrude(&outer, height);
	}

	// Layout positions: all loops' bottom ring, then all loops' top ring.
	let loops: Vec<&Vec<DVec2>> = std::iter::once(&outer).chain(holes.iter()).collect();
	let mut pos: Vec<DVec3> = Vec::new();
	let mut bottom_start = vec![0usize; loops.len()];
	let mut top_start = vec![0usize; loops.len()];
	for (li, lp) in loops.iter().enumerate() {
		bottom_start[li] = pos.len();
		pos.extend(lp.iter().map(|p| DVec3::new(p.x, p.y, 0.0)));
	}
	for (li, lp) in loops.iter().enumerate() {
		top_start[li] = pos.len();
		pos.extend(lp.iter().map(|p| DVec3::new(p.x, p.y, height)));
	}
	let ib = |li: usize, i: usize| (bottom_start[li] + i) as u32;
	let it = |li: usize, i: usize| (top_start[li] + i) as u32;
	let sgn = height.signum();
	let mut faces: Vec<FaceLoops> = Vec::new();

	// Bottom cap (faces −Z·sgn): outer loop reversed, holes forward.
	let mut blo = vec![(0..outer.len()).rev().map(|i| ib(0, i)).collect::<Vec<_>>()];
	for (hi, h) in holes.iter().enumerate() {
		blo.push((0..h.len()).map(|i| ib(hi + 1, i)).collect());
	}
	faces.push(FaceLoops { loops: blo, surface: Surface::Plane { origin: DVec3::ZERO, normal: -DVec3::Z * sgn } });

	// Top cap (faces +Z·sgn): outer loop forward, holes reversed.
	let mut bhi = vec![(0..outer.len()).map(|i| it(0, i)).collect::<Vec<_>>()];
	for (hi, h) in holes.iter().enumerate() {
		bhi.push((0..h.len()).rev().map(|i| it(hi + 1, i)).collect());
	}
	faces.push(FaceLoops { loops: bhi, surface: Surface::Plane { origin: DVec3::new(0.0, 0.0, height), normal: DVec3::Z * sgn } });

	// Outer walls (outward normal): [b[i], b[i+1], t[i+1], t[i]].
	let m = outer.len();
	for i in 0..m {
		let i1 = (i + 1) % m;
		let dir = outer[i1] - outer[i];
		let outward = DVec3::new(dir.y, -dir.x, 0.0).normalize_or_zero();
		let origin = DVec3::new(outer[i].x, outer[i].y, 0.0);
		faces.push(FaceLoops { loops: vec![vec![ib(0, i), ib(0, i1), it(0, i1), it(0, i)]], surface: Surface::Plane { origin, normal: outward } });
	}
	// Hole walls (normal points INTO the hole): reversed winding [b[i+1], b[i], t[i], t[i+1]].
	for (hi, h) in holes.iter().enumerate() {
		let li = hi + 1;
		let k = h.len();
		for i in 0..k {
			let i1 = (i + 1) % k;
			let dir = h[i1] - h[i];
			let inward = DVec3::new(-dir.y, dir.x, 0.0).normalize_or_zero();
			let origin = DVec3::new(h[i].x, h[i].y, 0.0);
			faces.push(FaceLoops { loops: vec![vec![ib(li, i1), ib(li, i), it(li, i), it(li, i1)]], surface: Surface::Plane { origin, normal: inward } });
		}
	}
	Solid::from_faces_multiloop(pos, faces)
}

/// A cylinder of `radius` and `height` (axis +Z, base at `z = 0`) whose **top rim is rounded** by
/// a fillet of `fillet` radius — the rounded-edge boss / pin / button-top that a curved-rim
/// (torus) fillet produces. Realised exactly as a surface of revolution: the cross-section is a
/// rectangle with one rounded top corner, so the result is watertight and genus-0 by construction,
/// no boolean needed. `segments` facets the revolution, `arc_segments` the fillet quarter-arc.
/// `fillet` is clamped to fit (`< radius` and `< height`); `fillet <= 0` yields a plain cylinder.
pub fn filleted_cylinder(radius: f64, height: f64, fillet: f64, segments: usize, arc_segments: usize) -> Solid {
	use std::f64::consts::FRAC_PI_2;
	let r = fillet.clamp(0.0, radius.min(height) * 0.999);
	if r <= 1e-12 {
		return cylinder(DVec3::ZERO, DVec3::Z, radius, height, segments);
	}
	let (seg, arc) = (segments.max(3), arc_segments.max(1));
	let axis = DVec3::Z;
	let (e1, e2) = perp_basis(axis);
	// Ring layers: layer 0 = bottom (R, z=0); layers 1..=arc = the fillet arc rings from the wall
	// tangent (R, z=h−r) to the cap tangent (R−r, z=h). Each layer is `seg` points around the axis.
	let mut pos = Vec::with_capacity((arc + 2) * seg);
	let push_ring = |rad: f64, z: f64, pos: &mut Vec<DVec3>| {
		for k in 0..seg {
			let a = TAU * k as f64 / seg as f64;
			pos.push((e1 * a.cos() + e2 * a.sin()) * rad + axis * z);
		}
	};
	push_ring(radius, 0.0, &mut pos);
	for k in 0..=arc {
		let psi = FRAC_PI_2 * k as f64 / arc as f64;
		push_ring((radius - r) + r * psi.cos(), (height - r) + r * psi.sin(), &mut pos);
	}
	let n_layers = arc + 2;
	let v = |l: usize, k: usize| (l * seg + (k % seg)) as u32;
	let cyl = Surface::Cylinder { origin: DVec3::ZERO, axis, radius };
	let tor = Surface::Torus { center: axis * (height - r), axis, major: radius - r, minor: r };
	let mut faces = Vec::with_capacity(n_layers * seg + 2);
	// Caps: bottom n-gon (radius R, z=0, −Z) and top n-gon (radius R−r, z=h, +Z).
	faces.push(FaceInput { boundary: orient((0..seg as u32).collect(), &pos, -axis), surface: Surface::Plane { origin: DVec3::ZERO, normal: -axis } });
	let top = ((n_layers - 1) * seg) as u32;
	faces.push(FaceInput { boundary: orient((top..top + seg as u32).collect(), &pos, axis), surface: Surface::Plane { origin: axis * height, normal: axis } });
	// Bands: layer 0→1 is the vertical wall (Cylinder); layers 1..=arc are the fillet (Torus).
	for l in 0..n_layers - 1 {
		let (surf, on_fillet) = if l == 0 { (cyl, false) } else { (tor, true) };
		for k in 0..seg {
			let k1 = (k + 1) % seg;
			let mid = TAU * (k as f64 + 0.5) / seg as f64;
			let radial = e1 * mid.cos() + e2 * mid.sin();
			let outward = if on_fillet {
				let psi = FRAC_PI_2 * (l as f64 - 0.5) / arc as f64;
				radial * psi.cos() + axis * psi.sin()
			} else {
				radial
			};
			let quad = vec![v(l, k), v(l, k1), v(l + 1, k1), v(l + 1, k)];
			faces.push(FaceInput { boundary: orient(quad, &pos, outward), surface: surf });
		}
	}
	Solid::from_faces(pos, faces).with_primitive_names()
}

/// A cylinder of `radius` and `height` (axis +Z, base at `z = 0`) whose **top rim is chamfered**
/// by a 45° bevel of size `chamfer` — the cut-edge counterpart of [`filleted_cylinder`]. Built as a
/// surface of revolution (the cross-section is a rectangle with one corner cut off), so it is
/// watertight and genus-0 by construction, no boolean. `segments` facets the revolution. `chamfer`
/// is clamped to fit (`< radius` and `< height`); `chamfer <= 0` yields a plain cylinder.
pub fn chamfered_cylinder(radius: f64, height: f64, chamfer: f64, segments: usize) -> Solid {
	let c = chamfer.clamp(0.0, radius.min(height) * 0.999);
	let mut profile = vec![DVec2::new(0.0, 0.0), DVec2::new(radius, 0.0)];
	if c > 1e-12 {
		// One straight bevel from the wall (radius, height−c) to the cap (radius−c, height) — a 45°
		// chamfer (equal radial and axial setback).
		profile.push(DVec2::new(radius, height - c));
		profile.push(DVec2::new(radius - c, height));
	} else {
		profile.push(DVec2::new(radius, height));
	}
	profile.push(DVec2::new(0.0, height));
	revolve(&profile, segments.max(3))
}

/// A **torus** (donut / O-ring) of `major` ring radius and `minor` tube radius, centred at
/// `center` with its ring lying in the plane perpendicular to `axis`. Every face carries the
/// analytic [`Surface::Torus`] tag, so the solid meshes to the TRUE torus under adaptive
/// tessellation (each facet projected onto the surface) and exports a `TOROIDAL_SURFACE`.
/// `ring_seg` facets the ring (around the axis); `tube_seg` facets the tube cross-section.
/// Empty when `minor >= major` (self-intersecting) or a radius is non-positive.
pub fn torus(center: DVec3, axis: DVec3, major: f64, minor: f64, ring_seg: usize, tube_seg: usize) -> Solid {
	if !major.is_finite() || major <= 0.0 || !minor.is_finite() || minor <= 0.0 || minor >= major {
		return Solid::default();
	}
	let (ring_seg, tube_seg) = (ring_seg.max(3), tube_seg.max(3));
	let axis = axis.normalize();
	let (e1, e2) = perp_basis(axis);
	let surf = Surface::Torus { center, axis, major, minor };
	// Grid: ring `i` (around the axis) × tube `j` (around the tube cross-section).
	let mut pos = Vec::with_capacity(ring_seg * tube_seg);
	for i in 0..ring_seg {
		let theta = TAU * i as f64 / ring_seg as f64;
		let er = e1 * theta.cos() + e2 * theta.sin();
		let tube_center = center + er * major;
		for j in 0..tube_seg {
			let psi = TAU * j as f64 / tube_seg as f64;
			pos.push(tube_center + (er * psi.cos() + axis * psi.sin()) * minor);
		}
	}
	let idx = |i: usize, j: usize| (((i % ring_seg) * tube_seg) + (j % tube_seg)) as u32;
	let mut faces = Vec::with_capacity(ring_seg * tube_seg);
	for i in 0..ring_seg {
		let theta = TAU * (i as f64 + 0.5) / ring_seg as f64;
		let er = e1 * theta.cos() + e2 * theta.sin();
		for j in 0..tube_seg {
			let psi = TAU * (j as f64 + 0.5) / tube_seg as f64;
			let outward = er * psi.cos() + axis * psi.sin();
			let quad = vec![idx(i, j), idx(i + 1, j), idx(i + 1, j + 1), idx(i, j + 1)];
			faces.push(FaceInput { boundary: orient(quad, &pos, outward), surface: surf });
		}
	}
	Solid::from_faces(pos, faces).with_primitive_names()
}

/// Full 360° revolution of a closed `profile` `(r, z)` (radius ≥ 0, height) about
/// the Z axis, faceted into `segments` sectors. Each profile edge sweeps a band
/// tagged with the matching analytic surface (Cylinder / Plane / Cone); profile
/// points on the axis (`r ≈ 0`) become poles, yielding triangle fans. Any
/// **simple** (non-self-intersecting) positive-area polygon revolves to a valid
/// closed manifold — convex or concave, multi-segment, either input winding — by
/// forcing the profile counter-clockwise and orienting every band from its edge's
/// right-of-travel normal. Rejected inputs (degenerate profile, negative radius,
/// an isolated on-axis apex that would pinch) yield an empty [`Solid`].
///
/// **Facet meridians** (plan later booleans around them): the curved bands are
/// faceted along meridians at `k · 360°/segments` for `k = 0..segments`,
/// starting at θ = 0 (the +X half-plane). Boolean features should respect this
/// grid — keep cutter side planes OFF the meridians and keep small embedded
/// unions from straddling one (the least-margin arrangement corner; run
/// [`crate::boolean_hazards`] to lint an op, and pick `segments` divisible by
/// a feature pattern's count so every copy shares one facet phase).
pub fn revolve(profile: &[DVec2], segments: usize) -> Solid {
	// Sanitize like `extrude` (drop coincident / closing-duplicate points; reject a
	// < 3-point or zero-area profile) so degenerate or explicitly-closed input yields
	// an empty Solid instead of broken topology — and never panics. Then force the
	// cross-section counter-clockwise in the (r, z) plane: with CCW winding the
	// interior lies to the LEFT of every edge, so the edge's right-of-travel
	// perpendicular IS the outward normal — exact for any simple polygon, concave
	// corners included.
	let profile = match sanitize_profile(profile) {
		Some(p) => to_ccw(p),
		None => return Solid::default(),
	};
	let n = profile.len();
	let on_axis = |i: usize| profile[i].x.abs() < 1e-9;
	// A negative radius would revolve to an inside-out (negative-volume) solid.
	if profile.iter().any(|p| p.x < -1e-9) {
		return Solid::default();
	}
	// An on-axis point with both profile neighbours OFF the axis is an isolated apex:
	// revolving it makes two cone tips meet at a single point — a non-manifold pinch
	// (odd Euler characteristic). Reject rather than emit a defective solid.
	for i in 0..n {
		if on_axis(i) && !on_axis((i + n - 1) % n) && !on_axis((i + 1) % n) {
			return Solid::default();
		}
	}
	let segments = segments.max(3);

	// Vertex index of profile point `i` at sector `c` (poles collapse to one id).
	let mut pole_id = vec![u32::MAX; n];
	let mut ring_base = vec![u32::MAX; n];
	let mut pos: Vec<DVec3> = Vec::new();
	for i in 0..n {
		if on_axis(i) {
			pole_id[i] = pos.len() as u32;
			pos.push(DVec3::new(0.0, 0.0, profile[i].y));
		} else {
			ring_base[i] = pos.len() as u32;
			for c in 0..segments {
				let a = TAU * c as f64 / segments as f64;
				pos.push(DVec3::new(profile[i].x * a.cos(), profile[i].x * a.sin(), profile[i].y));
			}
		}
	}
	let vid = |i: usize, c: usize| {
		if on_axis(i) {
			pole_id[i]
		} else {
			ring_base[i] + (c % segments) as u32
		}
	};

	let mut faces = Vec::new();
	for i in 0..n {
		let j = (i + 1) % n;
		let (ri, zi) = (profile[i].x, profile[i].y);
		let (rj, zj) = (profile[j].x, profile[j].y);
		// An edge running along the axis (both ends are poles) sweeps zero area.
		if on_axis(i) && on_axis(j) {
			continue;
		}
		// Outward normal in the (r, z) plane: the edge's right-of-travel perpendicular
		// (the CCW-forced profile keeps the interior on the left of i→j). NOT a
		// flip-away-from-the-centroid heuristic — that is wrong for concave profiles,
		// where an edge's outward side can face the centroid: an L-shaped flange
		// section had its two concave-corner bands emitted inside-out, leaving the
		// boundary rings unpaired (open seams, 2 shells, genus 98 at 96 segments).
		let o2 = DVec2::new(zj - zi, -(rj - ri)).normalize_or_zero();
		// Analytic surface swept by edge i→j.
		let surface = if (ri - rj).abs() < 1e-9 {
			Surface::Cylinder { origin: DVec3::new(0.0, 0.0, zi), axis: DVec3::Z, radius: ri }
		} else if (zi - zj).abs() < 1e-9 {
			Surface::Plane { origin: DVec3::new(0.0, 0.0, zi), normal: DVec3::Z }
		} else {
			// Cone through the two (r, z) points; apex where the line meets the axis.
			let t = ri / (ri - rj);
			let z_apex = zi + t * (zj - zi);
			let apex = DVec3::new(0.0, 0.0, z_apex);
			// Derive the half-angle from the endpoint with non-zero radius — the
			// other endpoint may be the apex itself, where both r and Δz vanish.
			let (r_far, z_far) = if ri.abs() >= rj.abs() { (ri, zi) } else { (rj, zj) };
			let axis = if z_far > z_apex { DVec3::Z } else { -DVec3::Z };
			let half_angle = (r_far / (z_far - z_apex).abs()).atan();
			Surface::Cone { apex, axis, half_angle }
		};
		for c in 0..segments {
			let c1 = c + 1;
			// Map the (r, z) outward normal to 3D at the band's mid sector.
			let mid = TAU * (c as f64 + 0.5) / segments as f64;
			let radial = DVec3::new(mid.cos(), mid.sin(), 0.0);
			let outward = radial * o2.x + DVec3::Z * o2.y;
			let poly: Vec<u32> = if on_axis(i) {
				vec![vid(i, c), vid(j, c), vid(j, c1)]
			} else if on_axis(j) {
				vec![vid(i, c), vid(j, c), vid(i, c1)]
			} else {
				vec![vid(i, c), vid(j, c), vid(j, c1), vid(i, c1)]
			};
			faces.push(FaceInput { boundary: orient(poly, &pos, outward), surface });
		}
	}
	Solid::from_faces(pos, faces)
}

/// Force a 2D polygon counter-clockwise (signed-area test). [`extrude`] and
/// [`revolve`] already sanitize their input, but campaign code kept re-writing
/// this helper for legibility when authoring profiles clockwise — promoted here
/// so profiles can be written in whichever order reads best.
pub fn force_ccw(profile: Vec<DVec2>) -> Vec<DVec2> {
	to_ccw(profile)
}

/// Prism over an annular sector: radii `[r_in, r_out]` (pass `r_in = 0.0` for a
/// full pie wedge), angles `[a0_deg, a1_deg]` about +Z from the +X axis, height
/// `z ∈ [z0, z1]`, with the arcs faceted at ≈`step_deg` per segment. The polar
/// cutter every bayonet / castellation / sector-pocket needs — its side faces
/// are exact planes through the axis direction, its arc faces are chordal
/// facets. (Keep the side planes off a target revolve's facet meridians — see
/// [`revolve`]'s meridian note and [`crate::boolean_hazards`].)
pub fn sector_prism(r_in: f64, r_out: f64, a0_deg: f64, a1_deg: f64, z0: f64, z1: f64, step_deg: f64) -> Solid {
	if r_out <= 0.0 || r_out <= r_in || (a1_deg - a0_deg).abs() < 1e-9 || (z1 - z0).abs() < 1e-12 {
		return Solid::default();
	}
	let step = step_deg.abs().max(0.05);
	let n = (((a1_deg - a0_deg).abs() / step).ceil() as usize).max(1);
	let arc = |r: f64, rev: bool, pts: &mut Vec<DVec2>| {
		for i in 0..=n {
			let f = if rev { (n - i) as f64 } else { i as f64 } / n as f64;
			let a = (a0_deg + (a1_deg - a0_deg) * f).to_radians();
			pts.push(DVec2::new(r * a.cos(), r * a.sin()));
		}
	};
	let mut pts = Vec::with_capacity(2 * n + 3);
	if r_in > 0.0 {
		arc(r_out, false, &mut pts);
		arc(r_in, true, &mut pts);
	} else {
		pts.push(DVec2::new(0.0, 0.0));
		arc(r_out, false, &mut pts);
	}
	extrude(&to_ccw(pts), z1 - z0).transformed(kernel_core::math::DAffine3::from_translation(DVec3::new(0.0, 0.0, z0)))
}

/// A guaranteed **right-handed** cylindrical frame at angle `theta_deg` about
/// +Z: local X → radial (r̂), local Y → tangential (t̂, the +θ direction),
/// local Z stays +Z, translated to radius `at_radius`, height `z`. Building
/// polar features by hand invites the det = −1 trap (r̂ × ẑ = −t̂ — one wrong
/// column order silently MIRRORS the part); this frame is checked by
/// construction (r̂ × t̂ = ẑ).
pub fn radial_frame(theta_deg: f64, at_radius: f64, z: f64) -> kernel_core::math::DAffine3 {
	let a = theta_deg.to_radians();
	let rhat = DVec3::new(a.cos(), a.sin(), 0.0);
	let that = DVec3::new(-a.sin(), a.cos(), 0.0);
	kernel_core::math::DAffine3::from_mat3_translation(
		kernel_core::math::DMat3::from_cols(rhat, that, DVec3::Z),
		rhat * at_radius + DVec3::Z * z,
	)
}
