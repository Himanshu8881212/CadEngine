// Copyright (c) LMCAD. Licensed under the MIT License.

//! B-rep → triangle [`Mesh`].
//!
//! Per face: planar faces are ear-clipped exactly; curved faces are subdivided
//! on their analytic surface (bilinear/barycentric interpolation snapped onto
//! the surface via [`Surface::project`]) so subdivided points lie exactly on the
//! true shape. Normals come from the analytic surface; winding is forced to
//! match the outward face normal. The result is welded into a shared-vertex
//! manifold mesh.
//!
//! # Merged curved faces: interior refinement (the boundary-ring contract, opened)
//!
//! Historically every curved face was triangulated **from its boundary ring
//! only** — correct for the chord facets primitives and booleans emit (the ring
//! IS the facet), but a face-count-collapsing pass ([`crate::recover`]) emits
//! merged faces whose ring spans a wide arc of the surface; a boundary-only
//! triangulation would replace the bulge with chords and silently lose volume
//! (the reason sphere/torus recovery used to be retag-only). Such faces are now
//! detected by [`merged_curved_ring`] — the ring is measurably non-planar AND
//! its warp dwarfs its own boundary-edge chord sag (a merged ring keeps the
//! original fine boundary vertices, so its edges hug the surface while its
//! interior bulges; a legacy warped facet's warp is the same order as its edge
//! sag) — and triangulated by [`refine_curved_ring`]: ear-clip the ring in the
//! surface's parameter chart ([`SurfaceChart`]), then split interior edges
//! (NEVER ring edges, so shared seams stay verbatim and the weld stays
//! watertight) at surface-projected midpoints until every interior chord is
//! within [`REFINE_REL_SAG`] of the surface (or the ring's own edge sag,
//! whichever is larger — a coarsely faceted input is not silently "improved"
//! past its own sampling). Planar-ring chord facets and facet-scale warped
//! rings keep the old paths byte-identically.

use kernel_core::math::{DVec2, DVec3};
use kernel_core::mesh::Mesh;
use kernel_core::orient2d;

use crate::geom::{perp_basis, Surface, SurfaceChart};
use crate::topo::Solid;

/// Tessellation controls.
#[derive(Clone, Copy, Debug)]
pub struct TessOptions {
	/// Subdivisions per direction for each curved face (1 = control facet only).
	pub curved_subdivisions: usize,
	/// Distance below which vertices are welded together.
	pub weld_tolerance: f32,
}

impl Default for TessOptions {
	fn default() -> Self {
		// 1 = use control corners only. This keeps shared edges between a curved
		// face and an adjacent planar face (e.g. a cylinder side and its cap)
		// coincident, so the welded mesh is watertight. Smoothness of curved
		// primitives is governed by their construction segment count. Raising
		// this is safe for solids whose faces are all the same surface. For a
		// *watertight* chord-tolerance tessellation use
		// [`crate::tessellate_adaptive_tol`], which subdivides shared edges
		// consistently.
		Self { curved_subdivisions: 1, weld_tolerance: 1e-5 }
	}
}

/// Tessellate a solid with default options.
pub fn tessellate_default(solid: &Solid) -> Mesh {
	tessellate(solid, &TessOptions::default())
}

/// Tessellate a solid into a welded triangle mesh.
pub fn tessellate(solid: &Solid, opts: &TessOptions) -> Mesh {
	let mut mesh = Mesh::new();
	for f in solid.faces() {
		let surface = solid.face(f).surface;
		let poly = solid.face_polygon(f);
		// Orientation is taken from the topological winding (the loop is already
		// outward-oriented), never from the surface-tag normal — so a planar tag
		// whose stored normal sign is incidental still tessellates correctly.
		let outward = newell_normal(&poly);
		match surface {
			Surface::Plane { .. } => {
				let inner = &solid.face(f).inner;
				if inner.is_empty() {
					tessellate_planar(&mut mesh, &poly, outward);
				} else {
					let holes: Vec<Vec<DVec3>> = inner.iter().map(|&lid| solid.loop_polygon(lid)).collect();
					tessellate_planar_with_holes(&mut mesh, &poly, &holes, outward);
				}
			}
			curved => tessellate_curved(&mut mesh, &poly, curved, opts.curved_subdivisions.max(1), outward),
		}
	}
	mesh.weld(opts.weld_tolerance);
	mesh
}

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

fn signed_area(p2: &[DVec2], idx: &[usize]) -> f64 {
	let mut a = 0.0;
	let n = idx.len();
	for i in 0..n {
		let c = p2[idx[i]];
		let d = p2[idx[(i + 1) % n]];
		a += c.x * d.y - d.x * c.y;
	}
	a * 0.5
}

fn point_in_tri(p: DVec2, a: DVec2, b: DVec2, c: DVec2) -> bool {
	// Exact orientation sign (`sign(p1,p2,p3) = orient2d(p3,p1,p2)`), so a vertex
	// near a candidate ear's edge classifies consistently instead of by a rounded f64.
	let sign = |p1: DVec2, p2: DVec2, p3: DVec2| orient2d([p3.x, p3.y], [p1.x, p1.y], [p2.x, p2.y]);
	let d1 = sign(p, a, b);
	let d2 = sign(p, b, c);
	let d3 = sign(p, c, a);
	let has_neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
	let has_pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
	if !(has_neg && has_pos) {
		return true; // strictly inside (or exactly on an edge)
	}
	// Also treat a vertex lying within a sub-weld distance of — and projecting onto — an
	// edge as "inside". A boolean's annular cap subdivides a straight rim into points that
	// are only *near*-collinear; without this an ear's base diagonal would skip such a point
	// (exact orient2d sees it just outside), the skipped point would later clip into an
	// overlapping sliver, and the seam would go non-manifold. Blocking the ear instead forces
	// the rim's subdivisions to be honoured, so the cap meshes watertight.
	[(a, b), (b, c), (c, a)].iter().any(|&(s, e)| near_segment(p, s, e))
}

/// Whether `p` lies within a sub-weld perpendicular distance of segment `s→e` and projects
/// onto its interior (the endpoints are the ear's own vertices, handled separately).
fn near_segment(p: DVec2, s: DVec2, e: DVec2) -> bool {
	const TOL: f64 = 1e-7; // well above f64 round-off (~1e-12), far below part feature size (mm)
	let se = e - s;
	let len2 = se.length_squared();
	if len2 < 1e-18 {
		return false;
	}
	let t = (p - s).dot(se) / len2;
	if !(0.0..=1.0).contains(&t) {
		return false;
	}
	(p - (s + se * t)).length() < TOL
}

fn tessellate_planar(mesh: &mut Mesh, poly: &[DVec3], normal: DVec3) {
	if poly.len() < 3 {
		return;
	}
	// A boolean-stitched annular cap arrives as ONE ring with the hole spliced
	// in through a doubled zero-width corridor (`bridge_hole_into`'s merge:
	// … P, M, hole…, M, P …). Ear-clipping that ring can roof the hole on large
	// concave outers (the shipped 60-tooth gear cap measured 18 wall crossings),
	// so recover the real outer + hole rings first and take the hole-aware path
	// with its verification ladder; a ring with no corridor is unchanged.
	if let Some((outer_ring, holes)) = unbake_keyholes(poly) {
		let holes3d: Vec<Vec<DVec3>> = holes.iter().map(|h| h.iter().map(|&i| poly[i]).collect()).collect();
		let outer3d: Vec<DVec3> = outer_ring.iter().map(|&i| poly[i]).collect();
		tessellate_planar_with_holes(mesh, &outer3d, &holes3d, normal);
		return;
	}
	let (u, v) = perp_basis(normal);
	let p2: Vec<DVec2> = poly.iter().map(|p| DVec2::new(p.dot(u), p.dot(v))).collect();
	ear_clip_ring(mesh, poly, &p2, (0..poly.len()).collect(), normal);
}

/// Detect and undo `bridge_hole_into`-style keyhole splices in a face polygon:
/// exact-duplicate vertex pairs `(i, j)` with `poly[i+1] == poly[j-1]` bracket a
/// spliced hole cycle `poly[i+1 .. j-1]`. Returns the outer ring plus every
/// recovered hole (indices into `poly`), or `None` when the ring carries no
/// corridor. Duplicates are exact bit-equal positions — the splice reuses the
/// welded vertex, so tolerance would only invite false positives.
pub(crate) fn unbake_keyholes(poly: &[DVec3]) -> Option<(Vec<usize>, Vec<Vec<usize>>)> {
	let n = poly.len();
	if n < 8 {
		return None; // smallest splice: 3-vertex outer + 3-vertex hole + 2 dups
	}
	let mut ring: Vec<usize> = (0..n).collect();
	let mut holes: Vec<Vec<usize>> = Vec::new();
	let eq = |a: usize, b: usize| poly[a] == poly[b];
	loop {
		let m = ring.len();
		let mut found: Option<(usize, usize)> = None;
		'scan: for i in 0..m {
			for j in (i + 3)..m {
				if eq(ring[i], ring[j]) && eq(ring[i + 1], ring[(j + m - 1) % m]) && j - i >= 4 {
					found = Some((i, j));
					break 'scan;
				}
			}
		}
		let Some((i, j)) = found else { break };
		// hole cycle: ring[i+1 ..= j-2] starts at the corridor mouth M and runs
		// the spliced hole once (M's closing duplicate at j-1 is dropped).
		let hole: Vec<usize> = ring[i + 1..j - 1].to_vec();
		if hole.len() < 3 {
			break; // degenerate corridor — leave the ring to the plain clip
		}
		let mut rest: Vec<usize> = Vec::with_capacity(ring.len() - hole.len() - 2);
		rest.extend_from_slice(&ring[..=i]);
		rest.extend_from_slice(&ring[j + 1..]);
		holes.push(hole);
		ring = rest;
		if ring.len() < 3 {
			return None; // over-stripped — not a keyhole pattern after all
		}
	}
	if holes.is_empty() {
		return None;
	}
	// Validate the decomposition before trusting it: dense rings (refined
	// revolve seams, sliver walls) can contain innocent duplicate-vertex
	// patterns that match the corridor signature without being keyholes, and a
	// false split flips winding over whole regions (measured: 105 non-orientable
	// edges on a horn at tol 0.01). A REAL keyhole hole lies strictly inside its
	// outer ring and is smaller than it; reject the unbake otherwise and let the
	// plain clip handle the ring unchanged.
	let n2 = |ring: &[usize]| -> (f64, f64, f64) {
		// Projected signed area (Newell z) + centroid in the dominant plane of
		// the polygon: adequate for containment screening on planar face rings.
		let mut area2 = 0.0;
		let (mut cx, mut cy) = (0.0, 0.0);
		let m = ring.len();
		for k in 0..m {
			let a = poly[ring[k]];
			let b = poly[ring[(k + 1) % m]];
			area2 += a.x * b.y - b.x * a.y;
			cx += a.x;
			cy += a.y;
		}
		(area2 * 0.5, cx / m as f64, cy / m as f64)
	};
	let inside = |x: f64, y: f64, ring: &[usize]| -> bool {
		let m = ring.len();
		let mut hit = false;
		for k in 0..m {
			let a = poly[ring[k]];
			let b = poly[ring[(k + 1) % m]];
			if (a.y > y) != (b.y > y) {
				let xi = a.x + (y - a.y) / (b.y - a.y) * (b.x - a.x);
				if xi > x {
					hit = !hit;
				}
			}
		}
		hit
	};
	let (outer_area, _, _) = n2(&ring);
	for hole in &holes {
		let (hole_area, hx, hy) = n2(hole);
		if hole_area.abs() >= outer_area.abs() || !inside(hx, hy, &ring) {
			return None; // not a keyhole decomposition — plain clip
		}
	}
	Some((ring, holes))
}

/// Ear-clip a CCW index `ring` into `poly` / `p2`, pushing the triangles. The ring
/// is reversed to CCW if needed; degenerate leftovers are fanned.
pub(crate) fn ear_clip_ring(mesh: &mut Mesh, poly: &[DVec3], p2: &[DVec2], idx: Vec<usize>, normal: DVec3) {
	ear_clip_ring_wound(mesh, poly, p2, idx, &|_| normal, &|_, _, _| normal)
}

/// [`ear_clip_ring`] with per-vertex normals and a per-triangle winding
/// reference. Required whenever the ring lives on a CURVED surface and spans a
/// wide arc: a single global reference is near-degenerate there and flips
/// triangles on the far side of the arc (the same failure the grid tessellator
/// had — measured 52 flipped directed edges on a transverse-bored cylinder's
/// bore wall). Planar callers keep the exact constant-normal behavior through
/// the wrapper above.
pub(crate) fn ear_clip_ring_wound(
	mesh: &mut Mesh,
	poly: &[DVec3],
	p2: &[DVec2],
	mut idx: Vec<usize>,
	nrm: &dyn Fn(DVec3) -> DVec3,
	wind: &dyn Fn(DVec3, DVec3, DVec3) -> DVec3,
) {
	if idx.len() < 3 {
		return;
	}
	if signed_area(p2, &idx) < 0.0 {
		idx.reverse(); // make CCW for the ear test
	}
	let mut guard = 0;
	while idx.len() > 3 && guard < 100_000 {
		guard += 1;
		let n = idx.len();
		let mut clipped = false;
		for i in 0..n {
			let ip = idx[(i + n - 1) % n];
			let ic = idx[i];
			let inx = idx[(i + 1) % n];
			let (a, b, c) = (p2[ip], p2[ic], p2[inx]);
			// Convex corner (CCW)? Exact orientation: a near-collinear corner is
			// classified reflex/flat consistently rather than by a rounded f64 sign.
			if orient2d([a.x, a.y], [b.x, b.y], [c.x, c.y]) <= 0.0 {
				continue;
			}
			// No other vertex inside this candidate ear? Vertices POSITIONALLY
			// identical to an ear corner are skipped: a keyhole corridor's twin
			// (the same point entering the ring twice, once per corridor side)
			// otherwise sits exactly ON the ear's edge and — through this
			// inclusive test — permanently blocks every corridor-adjacent ear,
			// stalling the whole clip into the guarded fan (measured: a
			// 12-hole plate's stalled remainder fanned slivers along the hole
			// rims — 111 non-orientable edges). A corridor EDGE poking through
			// the ear is still caught by the crossing test below.
			let mut ok = true;
			for &j in &idx {
				if j == ip || j == ic || j == inx || p2[j] == a || p2[j] == b || p2[j] == c {
					continue;
				}
				if point_in_tri(p2[j], a, b, c) {
					ok = false;
					break;
				}
			}
			// Vertex containment alone is NOT a sufficient ear test: a LONG ring
			// edge (a boolean's radial seam runs bore-to-outer in one span) can
			// slice through the candidate with both endpoints outside it. One
			// such ear roofed a gear bore and shipped 18 cap-to-wall crossings.
			// Reject the ear if any non-incident ring edge properly crosses any
			// of its three sides.
			if ok {
				'edges: for e in 0..n {
					let e0 = idx[e];
					let e1 = idx[(e + 1) % n];
					if e0 == ip || e0 == ic || e0 == inx || e1 == ip || e1 == ic || e1 == inx {
						continue;
					}
					let (s0, s1) = (p2[e0], p2[e1]);
					for (t0, t1) in [(a, b), (b, c), (c, a)] {
						if seg_proper_cross(t0, t1, s0, s1) {
							ok = false;
							break 'edges;
						}
					}
				}
			}
			if ok {
				let (a3, b3, c3) = (poly[ip], poly[ic], poly[inx]);
				push_tri(mesh, a3, b3, c3, nrm(a3), nrm(b3), nrm(c3), wind(a3, b3, c3));
				idx.remove(i);
				clipped = true;
				break;
			}
		}
		if !clipped {
			// Collinear-drain on stall: a flat corner (exact orient2d == 0 —
			// dense arcs produce runs of them) both blocks its neighbours' ears
			// through the inclusive point-in-triangle test and can never be an
			// ear itself. Removing ONE flat vertex loses no area and usually
			// unsticks the clip; repeat via the outer loop. Only a remainder
			// with no ear AND no flat vertex falls through to the fan — and a
			// CONCAVE remainder must never be fanned (the fan roofs concavities:
			// a stalled gear half-annulus measured 18 bore-wall crossings), so
			// the fan is now gated on the remainder being convex, with the
			// concave dead-end taking a centroid fan that at least stays inside
			// the polygon's kernel when one exists.
			let n = idx.len();
			let mut drained = false;
			for i in 0..n {
				let a = p2[idx[(i + n - 1) % n]];
				let b = p2[idx[i]];
				let c = p2[idx[(i + 1) % n]];
				if orient2d([a.x, a.y], [b.x, b.y], [c.x, c.y]) == 0.0 {
					idx.remove(i);
					drained = true;
					break;
				}
			}
			if !drained {
				break; // truly stuck; the guarded fan below decides
			}
		}
	}
	// Remaining triangle(s).
	if idx.len() == 3 {
		let (a3, b3, c3) = (poly[idx[0]], poly[idx[1]], poly[idx[2]]);
		push_tri(mesh, a3, b3, c3, nrm(a3), nrm(b3), nrm(c3), wind(a3, b3, c3));
		return;
	}
	if idx.len() < 3 {
		return;
	}
	// A convex remainder fans safely from any vertex; a concave one fans from
	// its 2D centroid (correct whenever the centroid sees the whole remainder,
	// and strictly better than an arbitrary-vertex fan in every case).
	let n = idx.len();
	let convex = (0..n).all(|i| {
		let a = p2[idx[(i + n - 1) % n]];
		let b = p2[idx[i]];
		let c = p2[idx[(i + 1) % n]];
		orient2d([a.x, a.y], [b.x, b.y], [c.x, c.y]) >= 0.0
	});
	if convex {
		for w in 1..n - 1 {
			let (a3, b3, c3) = (poly[idx[0]], poly[idx[w]], poly[idx[w + 1]]);
			push_tri(mesh, a3, b3, c3, nrm(a3), nrm(b3), nrm(c3), wind(a3, b3, c3));
		}
	} else {
		let c3 = idx.iter().fold(DVec3::ZERO, |acc, &i| acc + poly[i]) / n as f64;
		let c2 = idx.iter().fold(DVec2::ZERO, |acc, &i| acc + p2[i]) / n as f64;
		let _ = c2;
		for w in 0..n {
			let (a3, b3) = (poly[idx[w]], poly[idx[(w + 1) % n]]);
			push_tri(mesh, c3, a3, b3, nrm(c3), nrm(a3), nrm(b3), wind(c3, a3, b3));
		}
	}
}

/// Tessellate a planar face with inner hole loops (a washer / plate-with-holes).
/// Each hole is bridged into the outer loop (a doubled zero-width edge from the
/// hole's right-most vertex to a mutually-visible outer vertex), merging into one
/// simple polygon that is then ear-clipped — so the hole is a real cut, giving a
/// correct annular mesh and exact volume.
pub(crate) fn tessellate_planar_with_holes(mesh: &mut Mesh, outer3d: &[DVec3], holes3d: &[Vec<DVec3>], normal: DVec3) {
	if outer3d.len() < 3 {
		return;
	}
	let (u, v) = perp_basis(normal);
	let mut poly: Vec<DVec3> = outer3d.to_vec();
	for h in holes3d {
		poly.extend_from_slice(h);
	}
	let p2: Vec<DVec2> = poly.iter().map(|p| DVec2::new(p.dot(u), p.dot(v))).collect();

	let mut outer: Vec<usize> = (0..outer3d.len()).collect();
	if signed_area(&p2, &outer) < 0.0 {
		outer.reverse(); // outer CCW
	}
	let mut holes: Vec<Vec<usize>> = Vec::new();
	let mut start = outer3d.len();
	for h in holes3d {
		let mut ring: Vec<usize> = (start..start + h.len()).collect();
		if signed_area(&p2, &ring) > 0.0 {
			ring.reverse(); // holes CW (opposite the outer)
		}
		holes.push(ring);
		start += h.len();
	}
	// Bridge the right-most holes first so their bridges don't cross later ones.
	holes.sort_by(|a, b| ring_max_x(&p2, b).partial_cmp(&ring_max_x(&p2, a)).unwrap_or(std::cmp::Ordering::Equal));
	let all = holes.clone();
	// Retry ladder: the nearest-anchor bridge can self-overlap on large concave
	// outers (T15 family; a 60-tooth gear annulus measured 18 crossing pairs).
	// Each attempt bridges every hole with the attempt-th ranked anchor for the
	// FIRST hole (later holes stay nearest), clips into a scratch mesh, and
	// keeps the first attempt whose own triangles do not cross. Bounded and
	// deterministic; a fully exhausted ladder keeps attempt 0's output (the
	// historical behavior — downstream demotes the export to the voxel heal).
	const BRIDGE_ATTEMPTS: usize = 8;
	let mut kept: Option<Mesh> = None;
	let mut found_clean = false;
	for attempt in 0..BRIDGE_ATTEMPTS {
		let mut ring = outer.clone();
		for (hi, hole) in holes.iter().enumerate() {
			let skip = if hi == 0 { attempt } else { 0 };
			bridge_hole_into_ranked(&p2, &mut ring, hole, &all, skip);
		}
		let mut scratch = Mesh::new();
		ear_clip_ring(&mut scratch, &poly, &p2, ring.clone(), normal);
		// A valid cap neither crosses itself NOR covers a hole NOR lies about
		// its rim. The second test is the one the T15/gear family fails: a
		// mis-clipped ear can span the keyhole and roof the hole with triangles
		// whose crossings only appear against the hole's WALL — invisible to a
		// cap-only self-check. The third catches what BOTH miss: a stalled
		// clip's fan lays slivers flat ON the cap (they share vertices, so the
		// self-intersection sweep ignores them; they hug the rim, so the
		// hole-centroid test never sees them) — but any such sliver either
		// duplicates a directed edge or invents a rim the ring never had.
		let clean =
			!scratch.has_self_intersection() && !cap_covers_hole(&scratch, &p2, &holes, normal) && cap_rim_true(&scratch, &poly, &ring);
		if kept.is_none() || clean {
			kept = Some(scratch);
		}
		if clean {
			found_clean = true;
			break;
		}
	}
	// Deterministic last-resort for the annulus family: when every keyhole
	// attempt fails and the face is a single hole with both rings star-shaped
	// about the hole centroid (every gear/washer/flange cap), triangulate by
	// angular merge-strip — crossing-free by construction.
	if !found_clean && holes.len() == 1 {
		if let Some(strip) = annulus_strip(&poly, &p2, &outer, &holes[0], normal) {
			kept = Some(strip);
		}
	}
	if let Some(cap) = kept {
		let base = mesh.positions.len() as u32;
		mesh.positions.extend_from_slice(&cap.positions);
		mesh.normals.extend_from_slice(&cap.normals);
		mesh.indices.extend(cap.indices.iter().map(|i| i + base));
	}
}

/// Is the cap's own edge topology truthful? Keyed by EXACT vertex position
/// (the clip copies ring samples verbatim, so exact f32 equality is the right
/// join), the cap must (a) never traverse a directed edge twice — two
/// triangles walking the same edge the same way is a fold lying flat on the
/// cap — and (b) leave single-used (rim) edges only along the bridged ring's
/// own forward steps. Any other rim means the clip dropped ring vertices or
/// invented a chord: the face's neighbours still sample the true ring, so that
/// ships as a crack in the welded body even though the cap alone looks closed.
fn cap_rim_true(cap: &Mesh, poly: &[DVec3], ring: &[usize]) -> bool {
	type K = (u32, u32, u32);
	let key = |p: glam::Vec3| (p.x.to_bits(), p.y.to_bits(), p.z.to_bits());
	let mut dir: std::collections::HashMap<(K, K), u32> = std::collections::HashMap::new();
	for t in cap.indices.chunks_exact(3) {
		let (a, b, c) = (cap.positions[t[0] as usize], cap.positions[t[1] as usize], cap.positions[t[2] as usize]);
		for (e0, e1) in [(a, b), (b, c), (c, a)] {
			*dir.entry((key(e0), key(e1))).or_insert(0) += 1;
		}
	}
	let n = ring.len();
	let ringset: std::collections::HashSet<(K, K)> =
		(0..n).map(|k| (key(poly[ring[k]].as_vec3()), key(poly[ring[(k + 1) % n]].as_vec3()))).collect();
	dir.iter().all(|(&(ka, kb), &cnt)| {
		let paired = dir.get(&(kb, ka)).copied().unwrap_or(0) > 0;
		cnt == 1 && (paired || ringset.contains(&(ka, kb)))
	})
}

/// Does any triangle of `cap` roof a hole? True when a triangle centroid lands
/// strictly inside a hole ring — the signature of a keyhole mis-clip. Centroids
/// of LEGAL triangles can never sit inside a hole (the merged ring excludes the
/// hole's interior), so this is a pure rejection test with no false positives
/// beyond degenerate slivers already rejected elsewhere.
fn cap_covers_hole(cap: &Mesh, p2: &[DVec2], holes: &[Vec<usize>], normal: DVec3) -> bool {
	let (u, v) = perp_basis(normal);
	let project = |p: glam::Vec3| -> DVec2 {
		let d = DVec3::new(p.x as f64, p.y as f64, p.z as f64);
		DVec2::new(d.dot(u), d.dot(v))
	};
	let inside = |pt: DVec2, ring: &[usize]| -> bool {
		// Even-odd ray cast in 2D over the ring's projected vertices.
		let n = ring.len();
		let mut hit = false;
		for i in 0..n {
			let a = p2[ring[i]];
			let b = p2[ring[(i + 1) % n]];
			if (a.y > pt.y) != (b.y > pt.y) {
				let x = a.x + (pt.y - a.y) / (b.y - a.y) * (b.x - a.x);
				if x > pt.x {
					hit = !hit;
				}
			}
		}
		hit
	};
	for tri in cap.indices.chunks_exact(3) {
		let c = project(cap.positions[tri[0] as usize]) + project(cap.positions[tri[1] as usize]) + project(cap.positions[tri[2] as usize]);
		let c = c / 3.0;
		if holes.iter().any(|h| inside(c, h)) {
			return true;
		}
	}
	false
}

/// Angular merge-strip triangulation of a single-hole face whose outer and
/// hole rings are both star-shaped about the hole centroid: sweep both rings
/// by angle, always advancing the ring whose next vertex has the smaller
/// angle, emitting one triangle per advance. Returns `None` when either ring
/// is not strictly star-shaped (a fold-back would self-cross), leaving the
/// caller on the keyhole result.
fn annulus_strip(poly: &[DVec3], p2: &[DVec2], outer: &[usize], hole: &[usize], normal: DVec3) -> Option<Mesh> {
	if outer.len() < 3 || hole.len() < 3 {
		return None;
	}
	let centroid = hole.iter().fold(DVec2::ZERO, |a, &i| a + p2[i]) / hole.len() as f64;
	// Angle-sort both rings about the hole centroid; star-shapedness = the
	// sorted order is a rotation of ring order (no fold-backs).
	let sorted_ring = |ring: &[usize]| -> Option<Vec<usize>> {
		let mut with_angle: Vec<(f64, usize)> = ring.iter().map(|&i| ((p2[i] - centroid).y.atan2((p2[i] - centroid).x), i)).collect();
		let n = with_angle.len();
		// Strict monotonicity in ring order (up to one wrap) proves star shape.
		let mut wraps = 0;
		for k in 0..n {
			let a = with_angle[k].0;
			let b = with_angle[(k + 1) % n].0;
			if b <= a {
				wraps += 1;
			}
		}
		if wraps != 1 {
			return None;
		}
		with_angle.sort_by(|x, y| x.0.total_cmp(&y.0));
		Some(with_angle.into_iter().map(|(_, i)| i).collect())
	};
	let og = sorted_ring(outer)?;
	let hg = sorted_ring(hole)?;
	let angle = |i: usize| -> f64 {
		let d = p2[i] - centroid;
		d.y.atan2(d.x)
	};
	let mut mesh = Mesh::new();
	let (mut oi, mut hi) = (0usize, 0usize);
	let (on, hn) = (og.len(), hg.len());
	// March both rings once around, stitching the strip. Winding is enforced
	// by push_tri against the face normal, so sweep direction is irrelevant.
	while oi < on || hi < hn {
		let o0 = og[oi % on];
		let h0 = hg[hi % hn];
		let advance_outer = if oi >= on {
			false
		} else if hi >= hn {
			true
		} else {
			let oa = angle(og[(oi + 1) % on]);
			let ha = angle(hg[(hi + 1) % hn]);
			oa <= ha
		};
		if advance_outer {
			let o1 = og[(oi + 1) % on];
			push_tri(&mut mesh, poly[o0], poly[o1], poly[h0], normal, normal, normal, normal);
			oi += 1;
		} else {
			let h1 = hg[(hi + 1) % hn];
			push_tri(&mut mesh, poly[h0], poly[h1], poly[o0], normal, normal, normal, normal);
			hi += 1;
		}
	}
	Some(mesh)
}

/// Maximum x of a ring's projected vertices.
pub(crate) fn ring_max_x(p2: &[DVec2], ring: &[usize]) -> f64 {
	ring.iter().map(|&i| p2[i].x).fold(f64::NEG_INFINITY, f64::max)
}

/// Whether segments `a→b` and `c→d` cross at an interior point of both (proper,
/// no shared-endpoint touching).
fn seg_proper_cross(a: DVec2, b: DVec2, c: DVec2, d: DVec2) -> bool {
	let o = |p: DVec2, q: DVec2, r: DVec2| orient2d([p.x, p.y], [q.x, q.y], [r.x, r.y]);
	let (d1, d2, d3, d4) = (o(c, d, a), o(c, d, b), o(a, b, c), o(a, b, d));
	d1 * d2 < 0.0 && d3 * d4 < 0.0
}

/// True if the bridge segment from vertex `a` to vertex `b` crosses no ring edge
/// AND grazes no other ring vertex. The grazing check matters: a bridge passing
/// exactly through a third vertex is not a *proper* crossing, but splicing along
/// it still self-intersects the merged ring (the desk-mount plate meshed
/// non-manifold exactly this way — incidence-4 edges).
fn bridge_visible(p2: &[DVec2], a: usize, b: usize, outer: &[usize], holes: &[Vec<usize>]) -> bool {
	let (pa, pb) = (p2[a], p2[b]);
	let clear = |ring: &[usize]| {
		let n = ring.len();
		(0..n).all(|i| {
			let (c, d) = (ring[i], ring[(i + 1) % n]);
			(c == a || c == b || d == a || d == b || !seg_proper_cross(pa, pb, p2[c], p2[d]))
				&& (c == a || c == b || !near_segment(p2[c], pa, pb))
		})
	};
	clear(outer) && holes.iter().all(|h| clear(h))
}

/// Whether the point `target` lies strictly inside the face-interior wedge at
/// occurrence `pos` of `ring` (interior = left of the traversal, for the CCW
/// outer ring and CW hole rings alike). After earlier hole splices a vertex can
/// occur TWICE in the merged outer ring with different local wedges; bridging to
/// the wrong occurrence flips the new hole onto the wrong side of an earlier
/// bridge and double-covers a region, so the occurrence must pass this test.
fn dir_in_wedge(p2: &[DVec2], ring: &[usize], pos: usize, target: DVec2) -> bool {
	let n = ring.len();
	let o = |a: DVec2, b: DVec2, c: DVec2| orient2d([a.x, a.y], [b.x, b.y], [c.x, c.y]);
	let (prev, cur, next) = (p2[ring[(pos + n - 1) % n]], p2[ring[pos]], p2[ring[(pos + 1) % n]]);
	let (d1, d2) = (o(prev, cur, target), o(cur, next, target));
	if o(prev, cur, next) > 0.0 {
		d1 > 0.0 && d2 > 0.0
	} else {
		d1 > 0.0 || d2 > 0.0
	}
}

/// Splice `hole` (a CW ring) into the `outer` ring with a bridge from the hole's
/// right-most vertex to the nearest visible outer vertex, merging them into one ring.
/// `pub(crate)` so the boolean's loop-aware triangulation reuses the same bridging.
pub(crate) fn bridge_hole_into(p2: &[DVec2], outer: &mut Vec<usize>, hole: &[usize], all_holes: &[Vec<usize>]) {
	bridge_hole_into_ranked(p2, outer, hole, all_holes, 0);
}

/// [`bridge_hole_into`] with the `skip`-th ranked wedge-valid anchor instead of
/// the nearest: the retry ladder for caps whose nearest-anchor bridge produces
/// a self-overlapping triangulation (large concave outers — a 60-tooth gear
/// annulus was the shipped case). `skip: 0` is exactly the historical choice.
pub(crate) fn bridge_hole_into_ranked(p2: &[DVec2], outer: &mut Vec<usize>, hole: &[usize], all_holes: &[Vec<usize>], skip: usize) {
	if hole.is_empty() {
		return;
	}
	let m_local = (0..hole.len()).max_by(|&i, &j| p2[hole[i]].x.total_cmp(&p2[hole[j]].x)).unwrap();
	let m = hole[m_local];
	let mut candidates: Vec<(f64, usize)> = Vec::new();
	for (pos, &pv) in outer.iter().enumerate() {
		if pv == m
			|| !dir_in_wedge(p2, outer, pos, p2[m])
			|| !dir_in_wedge(p2, hole, m_local, p2[pv])
			|| !bridge_visible(p2, m, pv, outer, all_holes)
		{
			continue;
		}
		candidates.push(((p2[pv] - p2[m]).length_squared(), pos));
	}
	// Fall back to the old nearest-visible rule if the strict wedge tests reject
	// everything (a fully degenerate corner) — an imperfect bridge beats a
	// dropped hole.
	if candidates.is_empty() {
		for (pos, &pv) in outer.iter().enumerate() {
			if pv == m || !bridge_visible(p2, m, pv, outer, all_holes) {
				continue;
			}
			candidates.push(((p2[pv] - p2[m]).length_squared(), pos));
		}
	}
	candidates.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
	let Some(&(_, pos)) = candidates.get(skip.min(candidates.len().saturating_sub(1))) else {
		return; // no visible bridge (degenerate); leave the hole un-merged
	};
	// Insert: …outer[..=pos] (ends at P), hole from M all the way round, M again, P again, outer[pos+1..]…
	let mut seq: Vec<usize> = (0..hole.len()).map(|k| hole[(m_local + k) % hole.len()]).collect();
	seq.push(m);
	let mut merged = Vec::with_capacity(outer.len() + seq.len() + 1);
	merged.extend_from_slice(&outer[..=pos]);
	merged.extend_from_slice(&seq);
	merged.push(outer[pos]);
	merged.extend_from_slice(&outer[pos + 1..]);
	*outer = merged;
}

// --- Curved faces: surface-snapped subdivision -------------------------------

fn tessellate_curved(mesh: &mut Mesh, poly: &[DVec3], surface: Surface, subdiv: usize, face_outward: DVec3) {
	// Vertex normal from the analytic surface, sign-corrected to the face's
	// (topological) outward direction; winding always uses `face_outward`.
	let nrm = |p: DVec3| {
		let n = surface.normal_at(p);
		if n.dot(face_outward) < 0.0 {
			-n
		} else {
			n
		}
	};
	let outward = |_p: DVec3| face_outward;
	// `subdiv == 1` means "control facet only" (see `TessOptions`): emit the face's
	// own corners VERBATIM. The grid paths below project every grid point — at
	// subdiv 1 that projects the corners themselves, which is the identity for a
	// primitive facet (corners on the surface) but MOVES the cut corners of a
	// boolean fragment that still sit on a tessellation chord: each curved face
	// would shift its copy of the shared corner onto the surface while the planar
	// neighbour kept the original, cracking the weld (a cylinder∪cylinder result
	// meshed non-watertight). Smoothing belongs to `tessellate_adaptive`, whose
	// shared-edge subdivision moves both sides identically.
	if subdiv == 1 {
		tessellate_curved_verbatim(mesh, poly, surface, face_outward);
		return;
	}
	match poly.len() {
		4 => {
			let n = subdiv;
			let grid: Vec<Vec<DVec3>> = (0..=n)
				.map(|i| {
					let s = i as f64 / n as f64;
					(0..=n)
						.map(|j| {
							let t = j as f64 / n as f64;
							let a = poly[0].lerp(poly[1], s);
							let b = poly[3].lerp(poly[2], s);
							surface.project(a.lerp(b, t))
						})
						.collect()
				})
				.collect();
			for i in 0..n {
				for j in 0..n {
					let p00 = grid[i][j];
					let p10 = grid[i + 1][j];
					let p11 = grid[i + 1][j + 1];
					let p01 = grid[i][j + 1];
					push_tri(mesh, p00, p10, p11, nrm(p00), nrm(p10), nrm(p11), outward(p00));
					push_tri(mesh, p00, p11, p01, nrm(p00), nrm(p11), nrm(p01), outward(p00));
				}
			}
		}
		3 => {
			let (a, b, c) = (poly[0], poly[1], poly[2]);
			let n = subdiv;
			let pt = |i: usize, j: usize| {
				let bi = i as f64 / n as f64;
				let bj = j as f64 / n as f64;
				surface.project(a + (b - a) * bi + (c - a) * bj)
			};
			for i in 0..n {
				for j in 0..(n - i) {
					let p0 = pt(i, j);
					let p1 = pt(i + 1, j);
					let p2 = pt(i, j + 1);
					push_tri(mesh, p0, p1, p2, nrm(p0), nrm(p1), nrm(p2), outward(p0));
					if i + j + 2 <= n {
						let p3 = pt(i + 1, j + 1);
						push_tri(mesh, p1, p3, p2, nrm(p1), nrm(p3), nrm(p2), outward(p1));
					}
				}
			}
		}
		_ => {
			// A boolean-recovered curved facet with >4 vertices (e.g. a clipped bore band whose
			// straight cuts add collinear verts) is a SINGLE flat chord facet — its corners
			// already lie on the surface. Projecting and fanning from the centroid would bulge
			// it off its plane and self-intersect neighbouring facets, so tessellate it FLAT,
			// exactly as its planar twin would. The analytic surface tag still drives the
			// divergence-theorem exact_volume; only the preview/adaptive mesh stays chord-flat.
			tessellate_curved_verbatim(mesh, poly, surface, face_outward);
		}
	}
}

/// Triangulate a curved face's boundary VERBATIM (no interior points, corners
/// untouched — the watertightness contract of `curved_subdivisions = 1`), with
/// ONE exception: a **merged** curved face (see [`merged_curved_ring`]) gains
/// interior refinement points strictly inside its ring — the boundary is still
/// consumed verbatim, so shared seams stay coincident, but the bulge the ring
/// alone cannot represent is restored by [`refine_curved_ring`]. Otherwise: a
/// still-planar ring (every curved face before W5 seam snapping relaxed the
/// planarity contract) ear-clips in its plane, byte-identical to the old path; a
/// ring WARPED off its plane (seam-snapped vertices sit on the true intersection
/// curve, off the chord plane by up to the sagitta) ear-clips in the surface's
/// PARAMETER SPACE ([`SurfaceChart`]) instead — a projection-plane clip can fold
/// (self-intersect) on such a polygon and emit an unusable double-covered mesh.
fn tessellate_curved_verbatim(mesh: &mut Mesh, poly: &[DVec3], surface: Surface, face_outward: DVec3) {
	if merged_curved_ring(poly, &surface, face_outward) && push_refined_curved(mesh, poly, &surface, face_outward) {
		return;
	}
	if let Some(p2) = SurfaceChart::for_warped_ring(&surface, poly, face_outward).and_then(|c| c.uv_ring(poly)) {
		// Wide-arc warped ring: wind each ear against the analytic normal at its
		// own centroid, sign fixed once by the ring's aggregate vote (see
		// `push_refined_tris` — the single `face_outward` reference flips the
		// far side of the arc).
		let vote: f64 = poly.iter().map(|&p| surface.normal_at(p).dot(face_outward)).sum();
		let sigma = if vote < 0.0 { -1.0 } else { 1.0 };
		let nrm = move |p: DVec3| surface.normal_at(p) * sigma;
		let wind = move |a: DVec3, b: DVec3, c: DVec3| surface.normal_at((a + b + c) / 3.0) * sigma;
		ear_clip_ring_wound(mesh, poly, &p2, (0..poly.len()).collect(), &nrm, &wind);
		return;
	}
	tessellate_planar(mesh, poly, face_outward);
}

// --- Merged curved faces: interior-refined triangulation ----------------------

/// Warp-to-edge-sag ratio above which a non-planar curved ring counts as a
/// MERGED face (see [`merged_curved_ring`]). A legacy seam-snapped facet's ring
/// warp is the same order as its own edge chord sag (both are one facet's
/// sagitta); a merged face keeps the input mesh's fine boundary vertices, so
/// its edges hug the surface (sag ≈ one *original* facet) while the ring bows
/// off its plane by the *merged span's* sagitta — orders of magnitude more.
const MERGED_WARP_FACTOR: f64 = 16.0;

/// Angular span (radians) above which a warped curved ring counts as merged
/// even when its edge sag is not small (safety for e.g. a merged cap whose rim
/// is near-planar). Sits above every facet span the builders/booleans emit
/// (coarsest corpus builder: 6 segments → 2π/6 ≈ 1.05) and below every merged
/// chart span ([`crate::recover`] bins at ≥ π/2).
const MERGED_SPAN_MIN: f64 = 1.2;

/// Refinement FLOOR as a fraction of the surface's characteristic radius — a
/// runaway guard, not the fidelity target. The target is the face's own
/// boundary sagitta (see [`refine_tolerance`]): a merged face is made exactly
/// as faithful as the boundary it inherited, never silently finer (which would
/// drift it off its own chord input) and never coarser (which would lose the
/// bulge the merge exists to keep). This floor only stops an (almost) exactly
/// sampled boundary from demanding unbounded subdivision.
const REFINE_REL_SAG: f64 = 1e-5;

/// Maximum synchronized split rounds (each round quarters the worst sagitta).
const REFINE_MAX_ROUNDS: usize = 12;

/// Ring-index distance below which a boundary-to-boundary chord counts as a
/// local "ear" and is always split (see `short_ring_chord` in
/// [`refine_curved_ring`]) — the guard against a neighbouring face ear-clipping
/// the identical chord and welding it into a four-triangle edge.
const EAR_CHORD_RING_SPAN: usize = 8;

/// Hard cap on refined triangles per face (defensive; a full implicit-mesh
/// sphere face refines to a few tens of thousands).
const REFINE_MAX_TRIS: usize = 262_144;

/// Whether a curved face's boundary ring is a **merged** wide-span face that
/// needs interior refinement (see the module doc), as opposed to a legacy chord
/// facet (planar ring — the deliberate chord contract of primitives and
/// booleans) or a facet-scale seam-snapped warped ring (warp ≈ its own edge
/// sag). The discriminator is self-scaling: no absolute length enters except
/// the [`crate::geom::CURVED_WARP_EPS`]-scale floor that keeps exactly-planar
/// rings out.
pub(crate) fn merged_curved_ring(ring: &[DVec3], surface: &Surface, newell: DVec3) -> bool {
	if ring.len() < 8 || matches!(surface, Surface::Plane { .. }) || newell.length_squared() < 0.5 {
		return false;
	}
	let planarity = ring.iter().map(|&p| (p - ring[0]).dot(newell).abs()).fold(0.0, f64::max);
	if planarity <= 1e-6 {
		return false; // an exactly-planar ring is a deliberate chord facet
	}
	let edge_sag = max_ring_edge_sag(ring, surface);
	planarity > MERGED_WARP_FACTOR * edge_sag || crate::recover::angular_span(surface, ring) > MERGED_SPAN_MIN
}

/// Largest distance of any ring-edge chord midpoint from the surface.
fn max_ring_edge_sag(ring: &[DVec3], surface: &Surface) -> f64 {
	(0..ring.len())
		.map(|i| {
			let m = (ring[i] + ring[(i + 1) % ring.len()]) * 0.5;
			(surface.project(m) - m).length()
		})
		.fold(0.0, f64::max)
}

/// Margin on the boundary-sagitta target (see [`refine_tolerance`]). A face
/// whose interior chords ALREADY sag about as much as its own boundary chords
/// is left alone: without the margin the comparison is a coin-flip at equality,
/// and a band the STEP exporter coalesced out of ordinary chord facets (a
/// builder cylinder's wall arrives as two half-wrap faces) would gain interior
/// points on import and re-import 0.02% BULGED — breaking the exact
/// own-export round-trip. 1.5 is comfortably above that equality case and far
/// below the ratio a genuinely merged chart face shows (its boundary is one
/// original facet, its span hundreds).
const REFINE_BOUNDARY_MARGIN: f64 = 1.5;

/// The interior-chord sagitta the refinement drives every splittable edge
/// under: the larger of `REFINE_REL_SAG · r_char` (a floor) and the ring's
/// boundary edge sag times [`REFINE_BOUNDARY_MARGIN`] — a merged face is made
/// *as faithful as the boundary it inherited*, never silently finer (which
/// would drift it off its own chord input and off the volume the recover pass
/// gates against) and never coarser (which would lose the bulge the merge
/// exists to keep).
fn refine_tolerance(ring: &[DVec3], surface: &Surface) -> f64 {
	let r_char = match *surface {
		Surface::Plane { .. } => return f64::INFINITY,
		Surface::Cylinder { radius, .. } | Surface::Sphere { radius, .. } => radius,
		Surface::Cone { apex, axis, .. } => ring
			.iter()
			.map(|&p| {
				let d = p - apex;
				(d - axis * d.dot(axis)).length()
			})
			.fold(0.0, f64::max),
		Surface::Torus { major, minor, .. } => major + minor,
	};
	(REFINE_REL_SAG * r_char).max(REFINE_BOUNDARY_MARGIN * max_ring_edge_sag(ring, surface)).max(1e-9)
}

/// A face-local **invertible** parameter chart of a curved analytic surface —
/// the domain the merged-face refinement lives in. Distinct from
/// [`crate::geom::SurfaceChart`] (forward-only, for choosing ear-clip
/// diagonals) because refinement needs `uv → point`: the surface point at a
/// chart midpoint. Same anchoring discipline — every angle is unwrapped about
/// the ring's own mean direction, so a face spanning less than a full turn
/// never crosses a chart seam — and the same near-isometric scaling, so
/// [`refine_tolerance`] keeps its model-unit meaning.
///
/// - **Cylinder** → unrolled `(r·θ̃, z)`.
/// - **Sphere** → **gnomonic** about the ring's mean direction (injective on
///   the open hemisphere; a `recover` cubemap sextant spans ≲ 55° of it).
/// - **Cone** → isometric development `ρ·(cos, sin)(sin α · θ̃)`.
/// - **Torus** → `(R·θ̃, r·ψ̃)`, both angles unwrapped about ring means.
///
/// Injectivity is the caller's contract (the `recover` chart policy bins every
/// merged face to at most a half wrap per periodic direction); a ring that
/// violates it triangulates to a folded chart polygon, which the downstream
/// volume gate catches loudly rather than accepting.
#[derive(Clone, Copy, Debug)]
enum RefineChart {
	Cylinder { origin: DVec3, axis: DVec3, radius: f64, e1: DVec3, e2: DVec3 },
	Sphere { center: DVec3, radius: f64, w: DVec3, e1: DVec3, e2: DVec3 },
	Cone { apex: DVec3, axis: DVec3, half_angle: f64, e1: DVec3, e2: DVec3 },
	Torus { center: DVec3, axis: DVec3, major: f64, minor: f64, e1: DVec3, e2: DVec3, psi0: f64 },
}

impl RefineChart {
	/// Build the chart for `surface`, anchored on `ring`'s mean direction(s).
	/// `None` for a plane (no curvature to refine) or a degenerate ring (radial
	/// directions cancelling — a span this chart must not guess about).
	fn new(surface: &Surface, ring: &[DVec3]) -> Option<Self> {
		let mean_dir = |dirs: &mut dyn Iterator<Item = DVec3>| -> Option<DVec3> {
			let sum: DVec3 = dirs.fold(DVec3::ZERO, |a, d| a + d.normalize_or_zero());
			(sum.length_squared() > 1e-12).then(|| sum.normalize())
		};
		match *surface {
			Surface::Plane { .. } => None,
			Surface::Cylinder { origin, axis, radius } => {
				let axis = axis.normalize_or_zero();
				if axis.length_squared() < 0.5 || !(radius.is_finite() && radius > 0.0) {
					return None;
				}
				let e1 = mean_dir(&mut ring.iter().map(|&p| {
					let rel = p - origin;
					rel - axis * rel.dot(axis)
				}))?;
				Some(RefineChart::Cylinder { origin, axis, radius, e1, e2: axis.cross(e1) })
			}
			Surface::Sphere { center, radius } => {
				if !(radius.is_finite() && radius > 0.0) {
					return None;
				}
				let w = mean_dir(&mut ring.iter().map(|&p| p - center))?;
				let (e1, e2) = perp_basis(w);
				Some(RefineChart::Sphere { center, radius, w, e1, e2 })
			}
			Surface::Cone { apex, axis, half_angle } => {
				let axis = axis.normalize_or_zero();
				if axis.length_squared() < 0.5 || !(half_angle > 0.0 && half_angle < std::f64::consts::FRAC_PI_2) {
					return None;
				}
				let e1 = mean_dir(&mut ring.iter().map(|&p| {
					let rel = p - apex;
					rel - axis * rel.dot(axis)
				}))?;
				Some(RefineChart::Cone { apex, axis, half_angle, e1, e2: axis.cross(e1) })
			}
			Surface::Torus { center, axis, major, minor } => {
				let axis = axis.normalize_or_zero();
				if axis.length_squared() < 0.5 || !(major.is_finite() && major > 0.0 && minor.is_finite() && minor > 0.0) {
					return None;
				}
				let e1 = mean_dir(&mut ring.iter().map(|&p| {
					let rel = p - center;
					rel - axis * rel.dot(axis)
				}))?;
				let e2 = axis.cross(e1);
				// Mean tube direction in the (ring-radial, axis) plane — the ψ anchor.
				let psi_sum = ring.iter().fold(DVec2::ZERO, |a, &p| {
					let rel = p - center;
					let h = rel.dot(axis);
					let ring_dir = (rel - axis * h).normalize_or_zero();
					let d = p - (center + ring_dir * major);
					a + DVec2::new(d.dot(ring_dir), h).normalize_or_zero()
				});
				if psi_sum.length_squared() <= 1e-12 {
					return None;
				}
				Some(RefineChart::Torus { center, axis, major, minor, e1, e2, psi0: psi_sum.y.atan2(psi_sum.x) })
			}
		}
	}

	/// Chart coordinates of an on-surface point, or `None` outside the chart's
	/// injective domain (a gnomonic point at/behind the horizon, a point on the
	/// axis where the angle is undefined).
	fn uv(&self, p: DVec3) -> Option<DVec2> {
		let out = match *self {
			RefineChart::Cylinder { origin, axis, radius, e1, e2 } => {
				let rel = p - origin;
				let (x, y) = (rel.dot(e1), rel.dot(e2));
				if x * x + y * y < 1e-24 {
					return None;
				}
				DVec2::new(radius * y.atan2(x), rel.dot(axis))
			}
			RefineChart::Sphere { center, radius, w, e1, e2 } => {
				let rel = p - center;
				let d = rel.dot(w);
				if d <= 1e-9 * radius {
					return None; // at/behind the gnomonic horizon
				}
				DVec2::new(radius * rel.dot(e1) / d, radius * rel.dot(e2) / d)
			}
			RefineChart::Cone { apex, half_angle, e1, e2, .. } => {
				let rel = p - apex;
				let rho = rel.length();
				if rho < 1e-15 {
					return Some(DVec2::ZERO); // the apex develops to the origin exactly
				}
				let (x, y) = (rel.dot(e1), rel.dot(e2));
				if x * x + y * y < 1e-24 {
					return None;
				}
				let dev = half_angle.sin() * y.atan2(x);
				DVec2::new(rho * dev.cos(), rho * dev.sin())
			}
			RefineChart::Torus { center, axis, major, minor, e1, e2, psi0 } => {
				let rel = p - center;
				let (x, y) = (rel.dot(e1), rel.dot(e2));
				if x * x + y * y < 1e-24 {
					return None;
				}
				let h = rel.dot(axis);
				let rho = (rel - axis * h).length();
				let psi = h.atan2(rho - major) - psi0;
				let psi = psi - std::f64::consts::TAU * (psi / std::f64::consts::TAU).round(); // → (−π, π]
				DVec2::new(major * y.atan2(x), minor * psi)
			}
		};
		out.is_finite().then_some(out)
	}

	/// The surface point at chart coordinates `uv` — the exact inverse of
	/// [`Self::uv`] on its injective domain, and the reason this chart exists.
	fn point(&self, uv: DVec2) -> DVec3 {
		match *self {
			RefineChart::Cylinder { origin, axis, radius, e1, e2 } => {
				let t = uv.x / radius;
				origin + (e1 * t.cos() + e2 * t.sin()) * radius + axis * uv.y
			}
			RefineChart::Sphere { center, radius, w, e1, e2 } => {
				let dir = (w + e1 * (uv.x / radius) + e2 * (uv.y / radius)).normalize_or_zero();
				center + dir * radius
			}
			RefineChart::Cone { apex, axis, half_angle, e1, e2 } => {
				let rho = uv.length();
				if rho < 1e-15 {
					return apex;
				}
				let theta = uv.y.atan2(uv.x) / half_angle.sin();
				apex + axis * (rho * half_angle.cos()) + (e1 * theta.cos() + e2 * theta.sin()) * (rho * half_angle.sin())
			}
			RefineChart::Torus { center, axis, major, minor, e1, e2, psi0 } => {
				let theta = uv.x / major;
				let psi = psi0 + uv.y / minor;
				let ring = e1 * theta.cos() + e2 * theta.sin();
				center + ring * (major + minor * psi.cos()) + axis * (minor * psi.sin())
			}
		}
	}
}

/// Largest chart polygon the O(n²) greedy convex triangulation is run on
/// (386-vertex cylinder sector: ~75 k orientation tests, microseconds).
const GREEDY_CONVEX_MAX: usize = 4096;

/// Triangulate a **convex** chart polygon by repeatedly clipping the valid ear
/// with the SHORTEST diagonal — `None` when the polygon is not convex (or is
/// too large / degenerate).
///
/// Why greedy-shortest rather than the plain ear clip or a centroid fan: the
/// initial triangulation decides how much interior refinement the face then
/// needs, and both alternatives produce long diagonals across the face. On a
/// cylinder sector every diagonal spanning `Δθ` sags by `r(1−cos(Δθ/2))`
/// whatever its axial extent, so long diagonals force bisection rounds that end
/// COARSER than the face's own boundary sampling (measured: the 384-segment
/// detagged cylinder's merged sectors landed 6× further from `πr²h` than their
/// chord input). Shortest-diagonal clipping of the (rectangular) chart polygon
/// instead yields the quad STRIP between the two boundary chains — every
/// triangle spans one boundary step, so the merged face inherits the input's
/// fidelity exactly and needs no refinement at all.
///
/// Collinear corners are never clipped (they would emit zero-area triangles),
/// which is what keeps the densely-sampled straight chart edges intact.
fn greedy_convex_triangulation(p2: &[DVec2]) -> Option<Vec<[usize; 3]>> {
	let n = p2.len();
	if !(3..=GREEDY_CONVEX_MAX).contains(&n) {
		return None;
	}
	let idx0: Vec<usize> = (0..n).collect();
	let area = signed_area(p2, &idx0);
	if area == 0.0 {
		return None;
	}
	// Orient CCW, then require convexity: every corner turns left or is straight.
	let mut ring: Vec<usize> = idx0;
	if area < 0.0 {
		ring.reverse();
	}
	let turn = |a: usize, b: usize, c: usize| orient2d([p2[a].x, p2[a].y], [p2[b].x, p2[b].y], [p2[c].x, p2[c].y]);
	if (0..n).any(|i| turn(ring[(i + n - 1) % n], ring[i], ring[(i + 1) % n]) < 0.0) {
		return None; // a reflex corner: not convex, the ear test would need containment
	}
	let mut tris: Vec<[usize; 3]> = Vec::with_capacity(n.saturating_sub(2));
	while ring.len() > 3 {
		let m = ring.len();
		// Best ear = shortest diagonal among the non-degenerate (strictly convex)
		// corners; ties break to the lowest index, so the result is deterministic.
		let mut best: Option<(f64, usize)> = None;
		for i in 0..m {
			let (a, b, c) = (ring[(i + m - 1) % m], ring[i], ring[(i + 1) % m]);
			if turn(a, b, c) <= 0.0 {
				continue; // collinear (or degenerate): clipping would emit a sliver
			}
			let d = (p2[c] - p2[a]).length_squared();
			if best.is_none_or(|(bd, _)| d < bd) {
				best = Some((d, i));
			}
		}
		let (_, i) = best?; // a convex polygon always has a strictly convex corner
		let m = ring.len();
		tris.push([ring[(i + m - 1) % m], ring[i], ring[(i + 1) % m]]);
		ring.remove(i);
	}
	tris.push([ring[0], ring[1], ring[2]]);
	Some(tris)
}

/// Triangulate a chart polygon as a **validated star fan** from its centroid:
/// `Some((centroid, triangles))` when every boundary edge is counter-clockwise
/// as seen from the centroid — the fan then tiles the polygon exactly, in O(n),
/// which is what keeps a merged face with thousands of boundary samples (an
/// implicit-mesh sphere sextant) out of the ear clip's O(n³) worst case. The
/// centroid is emitted as index `p2.len()` (the caller appends its surface
/// point). `None` when the polygon is degenerate or not star-shaped about its
/// centroid — the caller falls back to the ear clip.
fn star_fan(p2: &[DVec2]) -> Option<(DVec2, Vec<[usize; 3]>)> {
	let n = p2.len();
	if n < 3 {
		return None;
	}
	let c = p2.iter().fold(DVec2::ZERO, |a, &p| a + p) / n as f64;
	if !c.is_finite() {
		return None;
	}
	// Orientation of the ring as a whole, then every edge against the centroid.
	let idx: Vec<usize> = (0..n).collect();
	let ccw = signed_area(p2, &idx) > 0.0;
	let center = n;
	let mut tris = Vec::with_capacity(n);
	for i in 0..n {
		let (a, b) = if ccw { (i, (i + 1) % n) } else { ((i + 1) % n, i) };
		let o = orient2d([c.x, c.y], [p2[a].x, p2[a].y], [p2[b].x, p2[b].y]);
		if o <= 0.0 {
			return None; // not star-shaped about the centroid (or a zero-area sliver)
		}
		tris.push([center, a, b]);
	}
	Some((c, tris))
}

/// Ear-clip a 2-D simple polygon into index triples (same exact-orientation ear
/// test as [`ear_clip_ring`], collecting indices instead of pushing triangles).
/// `None` when the polygon is degenerate (< 3 vertices or zero area).
fn ear_clip_indices(p2: &[DVec2]) -> Option<Vec<[usize; 3]>> {
	let mut idx: Vec<usize> = (0..p2.len()).collect();
	if idx.len() < 3 {
		return None;
	}
	if signed_area(p2, &idx) == 0.0 {
		return None;
	}
	if signed_area(p2, &idx) < 0.0 {
		idx.reverse();
	}
	let mut out: Vec<[usize; 3]> = Vec::with_capacity(idx.len().saturating_sub(2));
	let mut guard = 0;
	while idx.len() > 3 && guard < 100_000 {
		guard += 1;
		let n = idx.len();
		let mut clipped = false;
		for i in 0..n {
			let ip = idx[(i + n - 1) % n];
			let ic = idx[i];
			let inx = idx[(i + 1) % n];
			let (a, b, c) = (p2[ip], p2[ic], p2[inx]);
			if orient2d([a.x, a.y], [b.x, b.y], [c.x, c.y]) <= 0.0 {
				continue;
			}
			let mut ok = true;
			for &j in &idx {
				if j == ip || j == ic || j == inx {
					continue;
				}
				if point_in_tri(p2[j], a, b, c) {
					ok = false;
					break;
				}
			}
			if ok {
				out.push([ip, ic, inx]);
				idx.remove(i);
				clipped = true;
				break;
			}
		}
		if !clipped {
			break; // degenerate; fan the remainder below
		}
	}
	for w in 1..idx.len().saturating_sub(1) {
		out.push([idx[0], idx[w], idx[w + 1]]);
	}
	if out.is_empty() {
		None
	} else {
		Some(out)
	}
}

/// What [`refine_curved_ring`] hands back: the patch points (the ring verbatim
/// first, then interior points ON the surface), the triangle index triples, and
/// the per-triangle outward reference (the surface normal at each triangle's
/// chart centroid) callers must wind against.
pub(crate) type RefinedPatch = (Vec<DVec3>, Vec<[usize; 3]>, Vec<DVec3>);

/// Triangulate a **merged** curved face's boundary ring with interior
/// refinement, entirely in the surface's own parameter chart
/// ([`RefineChart`] — forward AND inverse, which is why this cannot reuse the
/// clip-only [`SurfaceChart`]):
/// 1. map the ring into the chart;
/// 2. triangulate the chart polygon — a validated star-shaped fan from its
///    centroid (O(n), the case every merged face hits) or, for a small
///    non-star polygon, the exact-orientation ear clip;
/// 3. repeatedly split every INTERIOR edge whose chord sags more than
///    [`refine_tolerance`], inserting the surface point at the edge's chart
///    MIDPOINT.
///
/// Refining in parameter space (rather than projecting the 3-D chord midpoint
/// with [`Surface::project`]) is load-bearing, not stylistic: a wide-span
/// face's triangulation contains chords whose 3-D midpoint lands on or near the
/// surface's axis, where `project`'s `normalize_or_zero` collapses and returns
/// the AXIS POINT — measured, a half-wrap cylinder sector so refined
/// triangulated to 2.5× its true area (watertight, and 31% over volume). The
/// chart midpoint is exact everywhere on the chart's injective domain.
///
/// Ring edges are never split (both endpoints on the ring at consecutive
/// positions), so the boundary is consumed verbatim and shared seams stay
/// watertight; splits are decided per undirected edge in synchronized rounds,
/// so the two triangles sharing an interior edge always split it together (no
/// T-junctions inside the face).
///
/// Returns `(points, triangles, outward)` — `points[..ring.len()]` are the ring
/// verbatim and the rest are interior points ON the surface; `outward[i]` is
/// the surface normal at triangle `i`'s own CHART centroid, the winding
/// reference callers must use. That reference is exact and never degenerate,
/// which matters: averaging the three vertex normals instead cancels to ~zero
/// on a triangle spanning a wide arc (a jagged merged face's ear clip emits
/// such triangles), and one randomly-flipped triangle is a non-manifold edge in
/// the welded mesh — measured on the recovered implicit cylinder. Projecting
/// the 3-D centroid is not an option either: that re-enters the axis
/// degeneracy this module exists to avoid.
///
/// `None` when the ring cannot be charted or triangulated (the caller falls
/// back to the boundary-only paths — and a volume gate downstream, e.g.
/// [`crate::recover`]'s 0.5% refusal, catches the fidelity loss loudly).
pub(crate) fn refine_curved_ring(ring: &[DVec3], surface: &Surface) -> Option<RefinedPatch> {
	let chart = RefineChart::new(surface, ring)?;
	let mut uv: Vec<DVec2> = ring.iter().map(|&p| chart.uv(p)).collect::<Option<Vec<_>>>()?;
	let n = ring.len();
	let mut pts: Vec<DVec3> = ring.to_vec();
	// Initial triangulation of the chart polygon, best-quality first: a convex
	// chart (every clean sector/quadrant) gets the strip-like greedy clip, a
	// jagged-but-star-shaped chart (a marching-cubes sphere sextant) gets the
	// O(n) centroid fan, anything else falls back to the exact ear clip.
	let mut tris = match greedy_convex_triangulation(&uv) {
		Some(t) => t,
		None => match star_fan(&uv) {
			Some((center_uv, fan)) => {
				uv.push(center_uv);
				pts.push(chart.point(center_uv));
				fan
			}
			None => ear_clip_indices(&uv)?,
		},
	};
	let tol = refine_tolerance(ring, surface);
	let ring_edge = |a: usize, b: usize| a < n && b < n && ((a + 1) % n == b || (b + 1) % n == a);
	// A chord joining two ring vertices that are CLOSE along the ring is a local
	// "ear" — and the neighbouring face across that stretch of boundary may well
	// ear-clip the very same chord, which welds into one edge used by four
	// triangles (a non-manifold edge in an otherwise closed mesh; measured on the
	// recovered implicit cylinder, where a merged cap and the merged wall both
	// spanned the same two rim vertices). Splitting such chords unconditionally
	// makes the duplication impossible: the interior point is this face's alone.
	// Long chords — the bottom-rim-to-top-rim diagonals of a clean strip
	// triangulation — are untouched, so an exactly-sampled face still emits its
	// chord facets verbatim.
	let short_ring_chord = |a: usize, b: usize| {
		if a >= n || b >= n {
			return false;
		}
		let d = a.abs_diff(b);
		d.min(n - d) <= EAR_CHORD_RING_SPAN
	};
	for _round in 0..REFINE_MAX_ROUNDS {
		// Decide splits once per undirected edge this round: the surface point at
		// the edge's CHART midpoint versus the edge's 3-D chord midpoint is the
		// chord sagitta; split when it exceeds `tol`.
		let mut split: std::collections::HashMap<(usize, usize), usize> = std::collections::HashMap::new();
		let mut new_pts: Vec<DVec3> = Vec::new();
		let mut new_uv: Vec<DVec2> = Vec::new();
		for t in &tris {
			for k in 0..3 {
				let (a, b) = (t[k], t[(k + 1) % 3]);
				let key = (a.min(b), a.max(b));
				if ring_edge(key.0, key.1) || split.contains_key(&key) {
					continue;
				}
				let mid_uv = (uv[a] + uv[b]) * 0.5;
				let on = chart.point(mid_uv);
				if (on - (pts[a] + pts[b]) * 0.5).length() > tol || short_ring_chord(key.0, key.1) {
					split.insert(key, pts.len() + new_pts.len());
					new_pts.push(on);
					new_uv.push(mid_uv);
				}
			}
		}
		if split.is_empty() || tris.len().saturating_mul(4) > REFINE_MAX_TRIS {
			break;
		}
		pts.extend(new_pts);
		uv.extend(new_uv);
		// Apply the standard 1/2/3-edge split patterns (deterministic diagonals).
		let mut next: Vec<[usize; 3]> = Vec::with_capacity(tris.len() * 2);
		for t in &tris {
			let [a, b, c] = *t;
			let m = |x: usize, y: usize| split.get(&(x.min(y), x.max(y))).copied();
			match (m(a, b), m(b, c), m(c, a)) {
				(None, None, None) => next.push([a, b, c]),
				(Some(mab), None, None) => next.extend([[a, mab, c], [mab, b, c]]),
				(None, Some(mbc), None) => next.extend([[b, mbc, a], [mbc, c, a]]),
				(None, None, Some(mca)) => next.extend([[c, mca, b], [mca, a, b]]),
				(Some(mab), Some(mbc), None) => {
					next.push([mab, b, mbc]);
					// Split the leftover quad (a, mab, mbc, c) by its shorter diagonal.
					if (pts[a] - pts[mbc]).length_squared() <= (pts[mab] - pts[c]).length_squared() {
						next.extend([[a, mab, mbc], [a, mbc, c]]);
					} else {
						next.extend([[a, mab, c], [mab, mbc, c]]);
					}
				}
				(None, Some(mbc), Some(mca)) => {
					next.push([mbc, c, mca]);
					if (pts[b] - pts[mca]).length_squared() <= (pts[mbc] - pts[a]).length_squared() {
						next.extend([[b, mbc, mca], [b, mca, a]]);
					} else {
						next.extend([[b, mbc, a], [mbc, mca, a]]);
					}
				}
				(Some(mab), None, Some(mca)) => {
					next.push([mca, a, mab]);
					if (pts[c] - pts[mab]).length_squared() <= (pts[mca] - pts[b]).length_squared() {
						next.extend([[c, mca, mab], [c, mab, b]]);
					} else {
						next.extend([[c, mca, b], [mca, mab, b]]);
					}
				}
				(Some(mab), Some(mbc), Some(mca)) => {
					next.extend([[a, mab, mca], [mab, b, mbc], [mca, mbc, c], [mab, mbc, mca]]);
				}
			}
		}
		tris = next;
	}
	// Per-triangle winding reference: the surface normal at the triangle's chart
	// centroid (exact, and defined everywhere on the chart's domain).
	let outward: Vec<DVec3> = tris
		.iter()
		.map(|t| {
			let c = (uv[t[0]] + uv[t[1]] + uv[t[2]]) / 3.0;
			surface.normal_at(chart.point(c))
		})
		.collect();
	Some((pts, tris, outward))
}

/// Push a merged face's refined triangulation into `mesh` with true per-vertex
/// surface normals, winding each triangle against the chart-centroid normal
/// [`refine_curved_ring`] hands back (a wide-span face's outward direction
/// varies across the face, so one ring normal cannot orient a hemisphere-scale
/// patch). The face's outward SIGN comes from the ring: for an outward-wound
/// loop the ring's Newell normal agrees with the surface normal. Returns
/// `false` (having pushed nothing) when the refinement could not run.
fn push_refined_curved(mesh: &mut Mesh, ring: &[DVec3], surface: &Surface, newell: DVec3) -> bool {
	let Some((pts, tris, outward)) = refine_curved_ring(ring, surface) else {
		return false;
	};
	push_refined_tris(mesh, &pts, &tris, &outward, ring, surface, newell);
	true
}

/// Shared emit half of [`push_refined_curved`] (also used by the adaptive
/// tessellator, whose boundary is the dense shared-seam polyline).
pub(crate) fn push_refined_tris(
	mesh: &mut Mesh,
	pts: &[DVec3],
	tris: &[[usize; 3]],
	outward: &[DVec3],
	ring: &[DVec3],
	surface: &Surface,
	newell: DVec3,
) {
	let sign = ring.iter().map(|&p| surface.normal_at(p).dot(newell)).sum::<f64>();
	let sigma = if sign < 0.0 { -1.0 } else { 1.0 };
	let nrm = |p: DVec3| surface.normal_at(p) * sigma;
	for (i, t) in tris.iter().enumerate() {
		let (a, b, c) = (pts[t[0]], pts[t[1]], pts[t[2]]);
		push_tri(mesh, a, b, c, nrm(a), nrm(b), nrm(c), outward[i] * sigma);
	}
}
