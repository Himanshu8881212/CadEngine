// Copyright (c) LMCAD. Licensed under the MIT License.

//! Exact curved cutting of a mesh by an analytic / implicit surface.
//!
//! The mesh-arrangement boolean ([`crate::booleans`]) is exact only for planar
//! faces: a curved cut there follows the tessellated facets, not the true surface.
//! This module trims a triangle mesh directly against an [`ImplicitSurface`],
//! placing every new boundary vertex *exactly* on that surface — found by Newton
//! along the crossing edge — instead of on a voxel grid. It is the building block
//! of an analytically exact curved boolean: a difference `A − B` by a convex tool
//! `B` is `trim_mesh_by_surface(A, ∂B, Keep::Outside)` stitched to the inside cap.

use std::collections::{HashMap, HashSet};

use std::f64::consts::PI;

use kernel_core::math::DVec3;
use kernel_core::mesh::Mesh;

use crate::geom::{perp_basis, Surface};
use crate::ssi::ImplicitSurface;

/// Which side of the surface to retain (`Inside` = where the field is negative).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Keep {
	Inside,
	Outside,
}

/// Trim `mesh` against `surf`, keeping the requested side. Triangles straddling
/// the surface are clipped so the new boundary vertices land on `surf` to `f64`
/// precision (stored at the mesh's `f32` resolution). The result is welded with
/// recomputed normals; its cut boundary is open (cap it separately for a solid).
pub fn trim_mesh_by_surface<S: ImplicitSurface + ?Sized>(mesh: &Mesh, surf: &S, keep: Keep) -> Mesh {
	let keep_sign = if keep == Keep::Inside { -1.0 } else { 1.0 };
	let mut out = Mesh::new();
	for tri in mesh.indices.chunks_exact(3) {
		let p = [
			mesh.positions[tri[0] as usize].as_dvec3(),
			mesh.positions[tri[1] as usize].as_dvec3(),
			mesh.positions[tri[2] as usize].as_dvec3(),
		];
		let d = [surf.value(p[0]), surf.value(p[1]), surf.value(p[2])];
		// Sutherland–Hodgman clip of the triangle against the kept half-space,
		// inserting the exact surface crossing wherever an edge changes side.
		let mut poly: Vec<DVec3> = Vec::with_capacity(4);
		for e in 0..3 {
			let (c, n) = (e, (e + 1) % 3);
			let (kc, kn) = (d[c] * keep_sign >= 0.0, d[n] * keep_sign >= 0.0);
			if kc {
				poly.push(p[c]);
			}
			if kc != kn {
				poly.push(refine_crossing(surf, p[c], p[n], d[c], d[n]));
			}
		}
		// Drop coincident consecutive clip points (including wrap-around): a triangle
		// vertex lying *on* the cut surface is emitted both as itself and as the edge
		// crossing through it, leaving a zero-width sliver that would weld into a
		// non-manifold edge. Removing the duplicates keeps the clipped polygon simple.
		poly.dedup_by(|x, y| (*x - *y).length() < 1e-6);
		while poly.len() >= 2 && (poly[0] - poly[poly.len() - 1]).length() < 1e-6 {
			poly.pop();
		}
		if poly.len() >= 3 {
			let base: Vec<u32> = poly.iter().map(|q| out.push_vertex(q.as_vec3())).collect();
			for k in 1..base.len() - 1 {
				out.push_triangle(base[0], base[k], base[k + 1]);
			}
		}
	}
	out.weld(1e-6);
	out.compute_normals();
	out
}

/// Ordered boundary (open-edge) loops of `mesh`, as cycles of vertex indices. A
/// boundary edge is a directed edge whose reverse is absent; for a clean
/// manifold-with-boundary each boundary vertex has a unique successor, so the edges
/// chain into closed loops. Applied to a [`trim_mesh_by_surface`] result this
/// recovers the exact cut seam — the analytic intersection curve, sampled on the
/// surface. (A pinch vertex shared by two loops is split arbitrarily.)
pub fn boundary_loops(mesh: &Mesh) -> Vec<Vec<u32>> {
	let mut dir: HashSet<(u32, u32)> = HashSet::new();
	for t in mesh.indices.chunks_exact(3) {
		dir.insert((t[0], t[1]));
		dir.insert((t[1], t[2]));
		dir.insert((t[2], t[0]));
	}
	// Outgoing boundary edges per vertex (those whose reverse is missing). A pinch
	// vertex — where two loops touch — has more than one, so use a multimap and
	// *consume* edges while walking, splitting the loops rather than dropping one.
	let mut out: HashMap<u32, Vec<u32>> = HashMap::new();
	for &(a, b) in &dir {
		if !dir.contains(&(b, a)) {
			out.entry(a).or_default().push(b);
		}
	}
	// Deterministic successor choice: the multimap above is filled in HashSet-random
	// order, so a pinch vertex (>1 successor) would split its two loops differently
	// run to run. Sorted, `pop()` always consumes the largest-id successor first.
	for succ in out.values_mut() {
		succ.sort_unstable();
	}
	let mut loops = Vec::new();
	let mut starts: Vec<u32> = out.keys().copied().collect();
	starts.sort_unstable(); // deterministic loop ordering
	for start in starts {
		while out.get(&start).is_some_and(|s| !s.is_empty()) {
			let mut loop_v = Vec::new();
			let mut v = start;
			while let Some(n) = out.get_mut(&v).and_then(|s| s.pop()) {
				loop_v.push(v);
				v = n;
				if v == start {
					break; // closed this loop
				}
			}
			if loop_v.len() >= 3 {
				loops.push(loop_v);
			}
		}
	}
	loops
}

/// The cut seam of a [`trim_mesh_by_surface`] result as ordered 3-D polylines —
/// the exact analytic intersection curve(s) where the surface met the mesh.
pub fn seam_loops(mesh: &Mesh) -> Vec<Vec<DVec3>> {
	boundary_loops(mesh).into_iter().map(|loop_v| loop_v.into_iter().map(|i| mesh.positions[i as usize].as_dvec3()).collect()).collect()
}

/// Close the open boundary loops of `mesh` with surface-following caps. Each loop is
/// filled by `rings` concentric rings interpolated toward `apex` and snapped back
/// onto the surface by `project`, so the cap follows the curved wall instead of
/// being a flat cone — the carved dimple is then shape-exact, not just vertex-exact.
/// `apex(centroid)` returns the cap pole (already on the surface). Each cap is a
/// closed 2-manifold patch reusing the reversed boundary edges.
fn cap_loops<A, P>(mesh: &mut Mesh, rings: usize, apex: A, project: P)
where
	A: Fn(DVec3) -> DVec3,
	P: Fn(DVec3) -> DVec3,
{
	let rings = rings.max(1);
	for loop_v in boundary_loops(mesh) {
		let n = loop_v.len();
		let pts: Vec<DVec3> = loop_v.iter().map(|&i| mesh.positions[i as usize].as_dvec3()).collect();
		let centroid = pts.iter().fold(DVec3::ZERO, |s, p| s + *p) / n as f64;
		let tip = apex(centroid);
		let mut prev = loop_v.clone();
		for m in 1..rings {
			let t = m as f64 / rings as f64;
			let cur: Vec<u32> = (0..n).map(|k| mesh.push_vertex(project(pts[k].lerp(tip, t)).as_vec3())).collect();
			for k in 0..n {
				let kn = (k + 1) % n;
				// Boundary edge prev[k]→prev[k+1]; the ring band reuses its reverse.
				mesh.push_triangle(prev[kn], prev[k], cur[k]);
				mesh.push_triangle(prev[kn], cur[k], cur[kn]);
			}
			prev = cur;
		}
		let a = mesh.push_vertex(tip.as_vec3());
		for k in 0..n {
			mesh.push_triangle(prev[(k + 1) % n], prev[k], a);
		}
	}
}

/// All three curved-boolean operations against a ball are a trim of the mesh
/// (keeping the part `keep` of the ball) closed by a spherical cap. They differ only
/// in `keep` and which pole anchors the cap: difference keeps the outside and caps
/// the inward dimple; union keeps the outside and caps the outward bump;
/// intersection keeps the inside and caps the inward wall.
fn ball_boolean(mesh: &Mesh, center: DVec3, radius: f64, keep: Keep, near_pole: bool) -> Mesh {
	let surf = Surface::Sphere { center, radius };
	let mut out = trim_mesh_by_surface(mesh, &surf, keep);
	let apex = |g: DVec3| {
		let off = g - center;
		let dir = if off.length() > 1e-9 * radius.max(1.0) { off.normalize() } else { DVec3::Z };
		// `dir` points from the ball centre out toward where it exits the mesh: the
		// near pole is the bump tip, the far pole the dimple bottom.
		center + dir * if near_pole { radius } else { -radius }
	};
	// Radial projection onto the sphere. When a ring point lands on the centre (a
	// seam point antipodal to the apex), the radial direction is undefined — fall
	// back to a fixed pole so the vertex stays on the sphere, never at 0.
	let project = |p: DVec3| {
		let off = p - center;
		let dir = if off.length_squared() > 1e-18 { off.normalize() } else { DVec3::Z };
		center + dir * radius
	};
	cap_loops(&mut out, 6, apex, project);
	out.compute_normals();
	out.ensure_outward();
	out
}

/// Subtract a ball from a mesh, returning a closed solid whose carved dimple lies on
/// the sphere — both the seam and the wall follow the sphere exactly (to the mesh's
/// `f32` resolution). A great-circle seam (tool centred on the surface) is ambiguous
/// and falls back to the `+Z` pole.
///
/// ```
/// use kernel_brep::{subtract_sphere, sphere, tessellate_default};
/// use kernel_brep::math::DVec3;
/// // Carve a bite out of a ball with a smaller overlapping ball; the result stays
/// // a closed solid with the bite wall lying on the cutting sphere.
/// let ball = tessellate_default(&sphere(DVec3::ZERO, 2.0, 24, 16));
/// let bitten = subtract_sphere(&ball, DVec3::X * 2.0, 1.0);
/// assert!(bitten.is_watertight());
/// ```
pub fn subtract_sphere(mesh: &Mesh, center: DVec3, radius: f64) -> Mesh {
	ball_boolean(mesh, center, radius, Keep::Outside, false)
}

/// Union a ball with a mesh, returning a closed solid whose added bump lies on the
/// sphere. Reuses the subtraction trim and cap, capping with the *outward* pole.
pub fn union_sphere(mesh: &Mesh, center: DVec3, radius: f64) -> Mesh {
	ball_boolean(mesh, center, radius, Keep::Outside, true)
}

/// Intersect a mesh with a ball, returning the (closed) common solid: the mesh
/// material inside the ball, walled by the spherical surface inside the mesh.
pub fn intersect_sphere(mesh: &Mesh, center: DVec3, radius: f64) -> Mesh {
	ball_boolean(mesh, center, radius, Keep::Inside, false)
}

/// Drill a cylindrical through-hole in a mesh. Trimming opens the bore as two seam
/// loops (entry and exit) on the cylinder; this stitches them into the tube wall.
/// The two rims wind oppositely about the axis, so `b` is reversed to align, then a
/// greedy angle zipper bands the two rims into a tube — handling rims of *unequal*
/// vertex count (oblique drills), advancing whichever side keeps the diagonal short.
/// The tube reuses each rim's reversed boundary edges, so the result is a
/// consistently-oriented closed solid. `axis` need not be unit-length.
pub fn drill_cylinder(mesh: &Mesh, origin: DVec3, axis: DVec3, radius: f64) -> Mesh {
	let axis = axis.normalize_or_zero();
	let surf = Surface::Cylinder { origin, axis, radius };
	let mut out = trim_mesh_by_surface(mesh, &surf, Keep::Outside);
	let loops = boundary_loops(&out);
	if loops.len() == 2 {
		let (e1, e2) = perp_basis(axis);
		let positions = out.positions.clone();
		let pos = |i: u32| positions[i as usize].as_dvec3();
		let ang_of = |i: u32| {
			let d = pos(i) - origin;
			d.dot(e2).atan2(d.dot(e1))
		};
		let a = loops[0].clone();
		let mut b = loops[1].clone();
		b.reverse(); // the two rims wind oppositely about the axis
			   // Rotate `b` so its start is angularly closest to `a[0]`.
		let a0 = ang_of(a[0]);
		let circ_dist = |x: f64| {
			let d = (x - a0).abs();
			if d > PI {
				2.0 * PI - d
			} else {
				d
			}
		};
		if let Some(start) = (0..b.len()).min_by(|&x, &y| circ_dist(ang_of(b[x])).total_cmp(&circ_dist(ang_of(b[y])))) {
			b.rotate_left(start);
		}
		// Greedy band: advance whichever rim's forward diagonal is shorter. Each step
		// reuses a's reversed edge (a[i+1]→a[i]) or b's edge (b[j]→b[j+1]).
		let (na, nb) = (a.len(), b.len());
		let (mut i, mut j) = (0usize, 0usize);
		for _ in 0..(na + nb) {
			let advance_a = if i == na {
				false
			} else if j == nb {
				true
			} else {
				let da = (pos(a[(i + 1) % na]) - pos(b[j % nb])).length();
				let db = (pos(a[i % na]) - pos(b[(j + 1) % nb])).length();
				da <= db
			};
			if advance_a {
				out.push_triangle(a[(i + 1) % na], a[i % na], b[j % nb]);
				i += 1;
			} else {
				out.push_triangle(a[i % na], b[j % nb], b[(j + 1) % nb]);
				j += 1;
			}
		}
	}
	out.compute_normals();
	out.ensure_outward();
	out
}

/// Subtract an (infinite, single-nappe) cone from a mesh, carving a conical pit
/// (countersink) whose wall lies on the cone. A cone is a ruled surface, so the cap
/// fans straight to its apex — every generator from apex to a seam vertex lies
/// exactly on the cone — and no ring subdivision is needed. `axis` points from the
/// apex into the body; `half_angle` is the cone's opening half-angle.
pub fn subtract_cone(mesh: &Mesh, apex: DVec3, axis: DVec3, half_angle: f64) -> Mesh {
	let surf = Surface::Cone { apex, axis: axis.normalize_or_zero(), half_angle };
	let mut out = trim_mesh_by_surface(mesh, &surf, Keep::Outside);
	cap_loops(&mut out, 1, |_| apex, |p| surf.project(p));
	out.compute_normals();
	out.ensure_outward();
	out
}

/// Bisection-refined zero crossing of the field along `a→b` (`da`, `db` are the
/// field values, of opposite sign). The returned point lies on the surface
/// (`value ≈ 0`), so it is the exact edge–surface intersection.
fn refine_crossing<S: ImplicitSurface + ?Sized>(surf: &S, mut a: DVec3, mut b: DVec3, mut da: f64, mut db: f64) -> DVec3 {
	let mut mid = a;
	for _ in 0..60 {
		let t = if (da - db).abs() > 1e-30 { (da / (da - db)).clamp(0.0, 1.0) } else { 0.5 };
		mid = a.lerp(b, t);
		let dm = surf.value(mid);
		if dm.abs() < 1e-12 {
			return mid;
		}
		if (da < 0.0) == (dm < 0.0) {
			a = mid;
			da = dm;
		} else {
			b = mid;
			db = dm;
		}
	}
	mid
}
