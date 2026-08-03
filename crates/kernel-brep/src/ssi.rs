// Copyright (c) LMCAD. Licensed under the MIT License.

//! Numerical surface–surface intersection (SSI).
//!
//! Traces the intersection curve(s) of two implicitly-defined surfaces as
//! polylines. Nothing here assumes a surface *type*: any field that supplies a
//! value (zero on the surface) and a gradient works, so the same marcher drives
//! NURBS–NURBS, analytic-quadric, and mixed intersections. The algorithm is the
//! classic implicit pattern — seed near the curve, Newton-project onto the joint
//! zero set `f = g = 0`, then step along the curve tangent `∇f × ∇g` and
//! re-project — all in `f64`.

use std::collections::HashMap;

use kernel_core::math::{DVec3, Vec3};
use kernel_core::mesh::Mesh;

use crate::geom::Surface;
use crate::nurbs::NurbsSurface;

/// A surface defined implicitly by a scalar field that is zero on the surface.
pub trait ImplicitSurface {
	/// Field value at `p`: zero on the surface, with opposite signs across it.
	fn value(&self, p: DVec3) -> f64;
	/// Gradient `∇value` at `p` (need not be unit; points off the surface).
	fn gradient(&self, p: DVec3) -> DVec3;
}

/// Drive the SSI directly from an analytic [`Surface`]: the value is the robust
/// signed implicit field (correct even on the medial axis, where a foot-point
/// projection degenerates), and the gradient is the outward normal at the foot
/// point — `∇(signed distance)` for a smooth surface.
impl ImplicitSurface for Surface {
	fn value(&self, p: DVec3) -> f64 {
		self.signed_value(p)
	}
	fn gradient(&self, p: DVec3) -> DVec3 {
		self.normal_at(self.project(p))
	}
}

/// Adapts a parametric [`NurbsSurface`] to the implicit interface by closest-point
/// projection: the signed distance to `p` is the foot-point offset along the
/// surface normal. A coarse `(u, v)` sample grid (built once) seeds a Gauss–Newton
/// point inversion so it lands on the *nearest* foot point rather than a far one.
pub struct NurbsField<'a> {
	surf: &'a NurbsSurface,
	samples: Vec<(f64, f64, DVec3)>,
}

impl<'a> NurbsField<'a> {
	/// Build the adapter, sampling a `seed × seed` grid over the surface domain.
	pub fn new(surf: &'a NurbsSurface, seed: usize) -> Self {
		let seed = seed.max(2);
		let ((u0, u1), (v0, v1)) = surf.domain();
		let mut samples = Vec::with_capacity(seed * seed);
		for i in 0..seed {
			let u = u0 + (u1 - u0) * i as f64 / (seed - 1) as f64;
			for j in 0..seed {
				let v = v0 + (v1 - v0) * j as f64 / (seed - 1) as f64;
				samples.push((u, v, surf.point_at(u, v)));
			}
		}
		Self { surf, samples }
	}

	/// Foot point `(u, v)`: nearest seed sample, refined by Gauss–Newton inversion
	/// of the orthogonality conditions `Sᵤ·(S−p) = Sᵥ·(S−p) = 0`.
	fn foot(&self, p: DVec3) -> (f64, f64) {
		let (mut u, mut v, _) = self.samples.iter().fold((0.0, 0.0, f64::INFINITY), |best, &(su, sv, s)| {
			let d = (s - p).length_squared();
			if d < best.2 {
				(su, sv, d)
			} else {
				best
			}
		});
		let ((u0, u1), (v0, v1)) = self.surf.domain();
		for _ in 0..32 {
			let r = self.surf.point_at(u, v) - p;
			let (du, dv) = self.surf.partials(u, v);
			let (f, g) = (du.dot(r), dv.dot(r));
			let (a, b, c) = (du.dot(du), du.dot(dv), dv.dot(dv));
			// `f`, `g` are in length² (`Sᵤ·r`), so scale the convergence test by the
			// partial-derivative magnitude — otherwise an absolute tolerance is
			// unreachable on large-coordinate surfaces and `foot` returns an
			// unconverged iterate (a badly wrong signed distance).
			let tol = 1e-13 * (a * c).sqrt().max(1.0);
			if f.abs() < tol && g.abs() < tol {
				break;
			}
			let det = a * c - b * b;
			if det.abs() < 1e-30 {
				break;
			}
			u = (u - (c * f - b * g) / det).clamp(u0, u1);
			v = (v - (a * g - b * f) / det).clamp(v0, v1);
		}
		(u, v)
	}
}

impl ImplicitSurface for NurbsField<'_> {
	fn value(&self, p: DVec3) -> f64 {
		let (u, v) = self.foot(p);
		(p - self.surf.point_at(u, v)).dot(self.surf.normal_at(u, v))
	}
	fn gradient(&self, p: DVec3) -> DVec3 {
		let (u, v) = self.foot(p);
		self.surf.normal_at(u, v)
	}
}

/// Tuning for [`intersect_surfaces`].
#[derive(Clone, Copy, Debug)]
pub struct SsiOptions {
	/// Seed-grid samples per axis over the domain box.
	pub seed_samples: usize,
	/// Marching step length in world units.
	pub step: f64,
	/// Newton convergence tolerance on `|value|`.
	pub tol: f64,
	/// Safety cap on points per traced polyline.
	pub max_points: usize,
}

impl Default for SsiOptions {
	fn default() -> Self {
		Self { seed_samples: 24, step: 0.1, tol: 1e-10, max_points: 100_000 }
	}
}

/// Snap a boolean's seam onto the *exact* intersection of two analytic surfaces.
///
/// A mesh∩mesh boolean (via [`crate::mesh_boolean`]) places its seam on the input
/// tessellation, so it is off the true surfaces by the facet chord error. For two
/// solids bounded by analytic [`ImplicitSurface`]s `f`, `g`, this projects every
/// vertex within `band` of *both* surfaces onto the exact `f ∩ g` curve with the
/// same min-norm Newton as the SSI tracer — making the seam analytically exact
/// (to the mesh's `f32` resolution) without any provenance tracking. Vertices not
/// near both surfaces are untouched; topology is unchanged, so a watertight input
/// stays watertight. Keep `band` near the tessellation error to avoid pulling the
/// seam's neighbourhood onto the curve.
pub fn snap_seam_to_intersection<F, G>(mesh: &mut Mesh, f: &F, g: &G, band: f64)
where
	F: ImplicitSurface + ?Sized,
	G: ImplicitSurface + ?Sized,
{
	for v in &mut mesh.positions {
		let p = v.as_dvec3();
		if f.value(p).abs() < band && g.value(p).abs() < band {
			if let Some(s) = project(f, g, p, 1e-10) {
				*v = s.as_vec3();
			}
		}
	}
}

/// Conformal (red–green) split of a triangle given the midpoint vertex on each of
/// its edges (`mids` = `[m_ab, m_bc, m_ca]`, `None` where the edge is not split).
/// Crack-free: a shared edge's single midpoint is used by both its triangles. The
/// canonical patterns below are CCW-preserving (verified by area/cross-product).
fn subdivide(tri: [u32; 3], mids: [Option<u32>; 3], out: &mut Vec<u32>) {
	let [a, b, c] = tri;
	let mut push = |x: u32, y: u32, z: u32| out.extend_from_slice(&[x, y, z]);
	match mids.iter().filter(|m| m.is_some()).count() {
		0 => push(a, b, c),
		1 => {
			// Rotate so the lone midpoint is on the first edge.
			let (a, b, c, m) = if let Some(m) = mids[0] {
				(a, b, c, m)
			} else if let Some(m) = mids[1] {
				(b, c, a, m)
			} else {
				(c, a, b, mids[2].unwrap())
			};
			push(a, m, c);
			push(m, b, c);
		}
		2 => {
			// Rotate so the empty edge is the third (midpoints on edges ab, bc).
			let (a, b, c, p, q) = if mids[2].is_none() {
				(a, b, c, mids[0].unwrap(), mids[1].unwrap())
			} else if mids[0].is_none() {
				(b, c, a, mids[1].unwrap(), mids[2].unwrap())
			} else {
				(c, a, b, mids[2].unwrap(), mids[0].unwrap())
			};
			push(a, p, c);
			push(p, b, q);
			push(p, q, c);
		}
		_ => {
			let (p, q, r) = (mids[0].unwrap(), mids[1].unwrap(), mids[2].unwrap());
			push(a, p, r);
			push(p, b, q);
			push(r, q, c);
			push(p, q, r);
		}
	}
}

/// Densify the seam of a boolean onto the exact `f ∩ g` curve: bisect every seam
/// edge (both endpoints within `band` of *both* surfaces), projecting the midpoint
/// onto the intersection, and conformally re-triangulate. Run after
/// [`snap_seam_to_intersection`] (so the seam vertices are already exact and `band`
/// can be tight). Topology stays a closed manifold; the seam region follows the
/// curve more densely. A vertex/edge not on the seam is untouched.
pub fn refine_seam_to_intersection<F, G>(mesh: &mut Mesh, f: &F, g: &G, band: f64)
where
	F: ImplicitSurface + ?Sized,
	G: ImplicitSurface + ?Sized,
{
	let pos: Vec<DVec3> = mesh.positions.iter().map(|p| p.as_dvec3()).collect();
	let on_seam = |i: u32| {
		let p = pos[i as usize];
		f.value(p).abs() < band && g.value(p).abs() < band
	};
	// One projected midpoint per seam edge (u32::MAX marks "no midpoint" — a checked
	// edge whose projection failed — so both its triangles agree and stay crack-free).
	let base = mesh.positions.len() as u32;
	let mut mids: HashMap<(u32, u32), u32> = HashMap::new();
	let mut new_pos: Vec<Vec3> = Vec::new();
	for t in mesh.indices.chunks_exact(3) {
		for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
			let key = if a < b { (a, b) } else { (b, a) };
			if mids.contains_key(&key) || !(on_seam(a) && on_seam(b)) {
				continue;
			}
			let m = (pos[a as usize] + pos[b as usize]) * 0.5;
			match project(f, g, m, 1e-10) {
				Some(s) => {
					mids.insert(key, base + new_pos.len() as u32);
					new_pos.push(s.as_vec3());
				}
				None => {
					mids.insert(key, u32::MAX);
				}
			}
		}
	}
	if new_pos.is_empty() {
		return;
	}
	let edge_mid = |a: u32, b: u32| {
		let k = if a < b { (a, b) } else { (b, a) };
		mids.get(&k).copied().filter(|&m| m != u32::MAX)
	};
	let old = std::mem::take(&mut mesh.indices);
	for t in old.chunks_exact(3) {
		subdivide([t[0], t[1], t[2]], [edge_mid(t[0], t[1]), edge_mid(t[1], t[2]), edge_mid(t[2], t[0])], &mut mesh.indices);
	}
	mesh.positions.extend(new_pos);
	mesh.compute_normals();
}

/// Min-norm Newton: slide `p` onto `f = g = 0` within the span of the two
/// gradients. Returns the converged point, or `None` if the gradients stay
/// parallel (tangential surfaces — no transverse curve) or it fails to converge.
/// `pub(crate)`: the boolean arrangement's seam snapper drives the same projector
/// to land cut-seam vertices on the exact surface–surface intersection.
pub(crate) fn project<F, G>(f: &F, g: &G, mut p: DVec3, tol: f64) -> Option<DVec3>
where
	F: ImplicitSurface + ?Sized,
	G: ImplicitSurface + ?Sized,
{
	for _ in 0..64 {
		let (fv, gv) = (f.value(p), g.value(p));
		if fv.abs() <= tol && gv.abs() <= tol {
			return Some(p);
		}
		let (gf, gg) = (f.gradient(p), g.gradient(p));
		// Solve [[m00,m01],[m01,m11]]·λ = −[fv;gv]; the min-norm step is gfᵀλ₀ + ggᵀλ₁.
		let (m00, m01, m11) = (gf.dot(gf), gf.dot(gg), gg.dot(gg));
		let det = m00 * m11 - m01 * m01;
		if det.abs() < 1e-30 {
			return None; // parallel gradients
		}
		let l0 = (-fv * m11 + gv * m01) / det;
		let l1 = (-gv * m00 + fv * m01) / det;
		p += gf * l0 + gg * l1;
		if !p.is_finite() {
			return None;
		}
	}
	let (fv, gv) = (f.value(p), g.value(p));
	(fv.abs() <= tol * 10.0 && gv.abs() <= tol * 10.0 && p.is_finite()).then_some(p)
}

/// Fully-determined Newton: slide `p` onto `f = g = h = 0` — a seam **corner**,
/// where three surfaces meet at a point (e.g. two cut planes crossing a curved
/// wall). Each step solves the exact 3×3 system `J·Δ = −F` whose rows are the
/// three gradients. Returns `None` when the gradients are (near-)linearly
/// dependent — no isolated triple point — or on non-convergence, so a degenerate
/// corner is left untouched rather than yanked.
pub(crate) fn project3<F, G, H>(f: &F, g: &G, h: &H, mut p: DVec3, tol: f64) -> Option<DVec3>
where
	F: ImplicitSurface + ?Sized,
	G: ImplicitSurface + ?Sized,
	H: ImplicitSurface + ?Sized,
{
	use kernel_core::math::DMat3;
	for _ in 0..64 {
		let vals = DVec3::new(f.value(p), g.value(p), h.value(p));
		if vals.x.abs() <= tol && vals.y.abs() <= tol && vals.z.abs() <= tol {
			return Some(p);
		}
		let (gf, gg, gh) = (f.gradient(p), g.gradient(p), h.gradient(p));
		// Matrix whose ROWS are the gradients (glam stores columns, so transpose).
		let m = DMat3::from_cols(
			DVec3::new(gf.x, gg.x, gh.x),
			DVec3::new(gf.y, gg.y, gh.y),
			DVec3::new(gf.z, gg.z, gh.z),
		);
		let det = m.determinant();
		if det.abs() < 1e-12 {
			return None; // gradients (near-)coplanar: no well-defined triple point
		}
		p += m.inverse() * (-vals);
		if !p.is_finite() {
			return None;
		}
	}
	let vals = DVec3::new(f.value(p), g.value(p), h.value(p));
	(vals.x.abs() <= tol * 10.0 && vals.y.abs() <= tol * 10.0 && vals.z.abs() <= tol * 10.0 && p.is_finite()).then_some(p)
}

/// Unit tangent of the intersection curve at `p` (perpendicular to both gradients).
fn tangent<F, G>(f: &F, g: &G, p: DVec3) -> Option<DVec3>
where
	F: ImplicitSurface + ?Sized,
	G: ImplicitSurface + ?Sized,
{
	let t = f.gradient(p).cross(g.gradient(p));
	(t.length() > 1e-14).then(|| t.normalize())
}

/// Is `p` inside the domain box grown by `margin` on every side?
fn in_box(p: DVec3, lo: DVec3, hi: DVec3, margin: f64) -> bool {
	p.x >= lo.x - margin
		&& p.x <= hi.x + margin
		&& p.y >= lo.y - margin
		&& p.y <= hi.y + margin
		&& p.z >= lo.z - margin
		&& p.z <= hi.z + margin
}

/// March one direction from `start`, projecting after each tangent step. Returns
/// the points (including `start`) and whether the curve closed back on `start`.
fn march<F, G>(f: &F, g: &G, start: DVec3, sign: f64, lo: DVec3, hi: DVec3, opts: &SsiOptions) -> (Vec<DVec3>, bool)
where
	F: ImplicitSurface + ?Sized,
	G: ImplicitSurface + ?Sized,
{
	let mut pts = vec![start];
	let mut p = start;
	let Some(mut prev_t) = tangent(f, g, start).map(|t| t * sign) else {
		return (pts, false);
	};
	for _ in 0..opts.max_points {
		let Some(mut t) = tangent(f, g, p) else { break };
		if t.dot(prev_t) < 0.0 {
			t = -t; // keep marching the same way around the curve
		}
		let Some(np) = project(f, g, p + t * opts.step, opts.tol) else { break };
		if !in_box(np, lo, hi, 0.0) || (np - p).length() < opts.step * 1e-3 {
			break; // left the domain, or stalled
		}
		if pts.len() > 3 && (np - start).length() < opts.step * 0.5 {
			return (pts, true); // closed loop
		}
		pts.push(np);
		prev_t = t;
		p = np;
	}
	(pts, false)
}

/// Trace the full polyline through `start`: march forward, and if it does not
/// close, march backward and prepend it.
fn trace<F, G>(f: &F, g: &G, start: DVec3, lo: DVec3, hi: DVec3, opts: &SsiOptions) -> Vec<DVec3>
where
	F: ImplicitSurface + ?Sized,
	G: ImplicitSurface + ?Sized,
{
	let (forward, closed) = march(f, g, start, 1.0, lo, hi, opts);
	if closed {
		return forward;
	}
	let (backward, _) = march(f, g, start, -1.0, lo, hi, opts);
	let mut line: Vec<DVec3> = backward.into_iter().skip(1).rev().collect();
	line.extend(forward);
	line
}

/// Trace the intersection polyline(s) of two implicit surfaces over the box
/// `[dmin, dmax]`. Each result is an ordered polyline; a closed curve's ends are
/// one marching step apart. Curves smaller than the seed spacing may be missed —
/// raise `seed_samples` (or shrink `step`) for fine features.
///
/// ```
/// use kernel_brep::{intersect_surfaces, SsiOptions, Surface};
/// use kernel_brep::math::DVec3;
/// // Two radius-5 spheres centred 6 apart meet in a single circle.
/// let a = Surface::Sphere { center: DVec3::ZERO, radius: 5.0 };
/// let b = Surface::Sphere { center: DVec3::X * 6.0, radius: 5.0 };
/// let opts = SsiOptions { seed_samples: 16, step: 0.1, ..Default::default() };
/// let loops = intersect_surfaces(&a, &b, DVec3::splat(-6.0), DVec3::splat(6.0), &opts);
/// assert_eq!(loops.len(), 1);
/// ```
pub fn intersect_surfaces<F, G>(f: &F, g: &G, dmin: DVec3, dmax: DVec3, opts: &SsiOptions) -> Vec<Vec<DVec3>>
where
	F: ImplicitSurface + ?Sized,
	G: ImplicitSurface + ?Sized,
{
	let n = opts.seed_samples.max(2);
	let span = dmax - dmin;
	let inv = 1.0 / (n - 1) as f64;
	let mut polylines: Vec<Vec<DVec3>> = Vec::new();
	let mut covered: Vec<DVec3> = Vec::new();
	for iz in 0..n {
		for iy in 0..n {
			for ix in 0..n {
				let frac = DVec3::new(ix as f64, iy as f64, iz as f64) * inv;
				let Some(p0) = project(f, g, dmin + span * frac, opts.tol) else { continue };
				if !in_box(p0, dmin, dmax, opts.step) {
					continue;
				}
				if covered.iter().any(|q| (*q - p0).length() < opts.step) {
					continue; // already on a traced curve
				}
				let line = trace(f, g, p0, dmin, dmax, opts);
				if line.len() >= 2 {
					covered.extend_from_slice(&line);
					polylines.push(line);
				}
				// A seed that failed to trace (e.g. it projected just outside the
				// strict domain) is deliberately NOT marked covered: doing so could
				// suppress a genuine interior seed nearby and drop a boundary-grazing
				// curve.
			}
		}
	}
	polylines
}
