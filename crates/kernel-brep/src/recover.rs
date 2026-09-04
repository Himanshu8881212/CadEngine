// Copyright (c) LMCAD. Licensed under the MIT License.

//! **Analytic quadric recovery** (reverse bridge v2) — [`coalesce_coplanar`]
//! generalized from planes to quadrics: a FINISHING PASS that finds connected
//! regions of planar chord facets which lie, within a caller tolerance, on one
//! cylinder / sphere / cone / torus (or one tolerant plane), and hands the
//! region that analytic [`Surface`] carrier.
//!
//! [`coalesce_coplanar`]: crate::coalesce::coalesce_coplanar
//!
//! # What it does, exactly
//! 1. **Region growing** — union-find over shared edges (the same discipline
//!    as `coalesce_coplanar`), but joining facets whose dihedral *bend* is in
//!    the smooth-curvature band `[BEND_MIN, BEND_MAX]`: exactly-coplanar seams
//!    stay plane territory (that is `coalesce_coplanar`'s job) and sharp
//!    feature creases (a cap rim, a prism corner) are never crossed.
//! 2. **Fitting** — deterministic least squares per region over the carrier
//!    family plane → cylinder → sphere → cone → torus. Phase 1 fits every
//!    kind to the WHOLE region and takes the lowest residual within `tol`
//!    (so a peeling cylinder can never steal a thin equator band of a genuine
//!    torus); only when no whole-region carrier exists does phase 2 peel
//!    facets whose own samples exceed `tol` and refit, kind by kind — a
//!    rounded crease band cannot poison a clean cylinder next to it, and
//!    peeled facets stay honest planar facets. Acceptance is judged on
//!    **sagitta-aware samples** (vertices + facet edge midpoints +
//!    centroids): a hexagonal prism's six flats have all vertices exactly ON
//!    the circumscribed cylinder, but its edge midpoints sit
//!    `r·(1−cos 30°) ≈ 0.134·r` off it — a 6-facet ring is NOT a cylinder at
//!    any tight tolerance, and the midpoint samples are what say so. Curved
//!    carriers must additionally subtend a real arc of their own surface
//!    ([`MIN_ARC`] guard): without it a mesher cap's near-flat rim ring
//!    "fits" a 365 mm cylinder lying on its side (measured). The TOLERANT
//!    PLANE is deliberately in the family: mesher caps arrive as near-flat
//!    facet fields the exact-key `coalesce_coplanar` cannot merge, and their
//!    carrier recovery is the same operation at zero curvature (counted
//!    separately in [`RecoveryReport::planes`] — a plane is not a quadric).
//! 3. **Rebuild** — no vertex is ever moved: recovery changes the surface
//!    *carrier*, not the point set.
//!    - **Tolerant-plane regions** merge whole (flat chords are exact, so no
//!      span budget applies, and a merged planar face may keep hole loops).
//!    - **Curved regions** MERGE into **chart faces** via the
//!      `coalesce_coplanar` region-boundary half-edge walk: interior facet
//!      edges disappear, boundary vertices stay exactly. The tessellators
//!      refine a merged face with INTERIOR points projected onto its analytic
//!      surface (see the `tessellate` module doc — the old boundary-ring-only
//!      contract is opened for merged faces), so the merge no longer loses the
//!      bulge volume. **Chart policy** (each merged face must chart injectively
//!      into its surface's parameter space, and a single-loop face cannot be a
//!      full periodic wrap): single-curved regions (cylinder, cone) split into
//!      TWO azimuth half-wrap sectors (span π — a full cylinder collapses to 2
//!      lateral faces + its caps, not to 1 periodic face); sphere regions split
//!      into up to SIX cubemap sextants (dominant-axis bins — each subtends
//!      ≲ 55° of gnomonic chart, comfortably injective); torus regions split
//!      into a 4 × 4 (azimuth × tube-angle) quadrant grid. A full-wrap sphere
//!      or torus therefore legitimately stays split into charts — face-count
//!      collapse to single digits, not to 1.
//!    - **Fallback cascade**, honest and deterministic: if the merged rebuild
//!      fails a gate below, the pass retries with doubly-curved regions
//!      RETAGGED facet-by-facet (carrier without collapse), then with the
//!      legacy 0.11-rad span-budgeted single-curved sectors — so a solid the
//!      chart merge cannot represent volume-faithfully degrades to the
//!      previous, weaker-but-safe behavior instead of refusing outright. The
//!      pinned face counts in the tests make a silent descent visible.
//! 4. **Gates** — the rebuilt solid must pass [`validate`] and its
//!    [`volume`] (default tessellation) must stay within `DRIFT_MAX` (0.5%) of
//!    the input's, else the pass REFUSES with the measured numbers. Running on
//!    an already-analytic solid (a builder cylinder) is a structural no-op.
//!
//! # Residual, reported not hidden
//! [`RecoveryReport::max_fit_residual`] is the maximum distance of any
//! recovered region's facet VERTICES from its fitted surface — a superset of
//! the kept boundary vertices, so it is conservative for the rebuilt solid.
//! Fit *acceptance* additionally samples edge midpoints and centroids (the
//! chord sagitta), which is strictly harder to pass.
//!
//! # Scope, stated honestly
//! - **Provenance survives the rebuild** (FRICTION #20's residual half,
//!   LIFTED): an unmerged face keeps its [`crate::topo::FaceName`] exactly; a
//!   merged face inherits the lexicographically-least constituent name (the
//!   policy documented on [`crate::topo::FaceName`]); analytic edge curves
//!   whose endpoints both survive are re-attached. The pass may therefore run
//!   MID-CHAIN — witness-addressed features re-resolve afterwards. Remaining
//!   caveat, stated: the names of fully-consumed interior fragments (and the
//!   non-least names of a multi-name merge) no longer resolve — they name
//!   geometry that no longer exists as a face, which is correct. A solid with
//!   no provenance at all stays that way.
//! - Carrier selection is deterministic, not oracular: phase 1 takes the
//!   lowest whole-region residual, phase 2 the largest explained facet count
//!   (ties toward plane → cylinder → sphere → cone → torus). A shallow cone
//!   slice whose best whole-region carrier is a cylinder within `tol` is
//!   carried as that cylinder — a within-tolerance carrier, never silent.
//! - Peeled facets are not re-admitted after the fit converges; they remain
//!   planar facets (visible as `faces_after` not reaching the ideal count).

use std::collections::BTreeSet;

use kernel_core::math::{DMat3, DVec3};

use crate::geom::{perp_basis, Surface};
use crate::topo::{FaceId, FaceLoops, FaceName, Solid};
use crate::validate::{validate, volume};

/// Result of one [`recover_quadrics`] pass. Counts are recovered REGIONS (one
/// region may rebuild into several sector faces), `faces_before/after` are the
/// solid's face counts, and `max_fit_residual` is the maximum distance of any
/// recovered region's facet vertices from its fitted surface (see module doc;
/// 0.0 when nothing was recovered).
///
/// `planes` counts TOLERANT-PLANE regions: mesher output carries near-flat
/// facet fields (a dual-contour cap's rim ring tilts by fractions of a degree,
/// so the exact-key [`crate::coalesce_coplanar`] cannot merge it) and
/// recovering their plane carrier is the same fit-within-`tol` operation at
/// zero curvature — without it the cap noise dominates the face count and the
/// STEP payoff. A plane is not a quadric; the field is separate so the quadric
/// counts stay honest.
#[derive(Clone, Copy, Debug, PartialEq, Default)]
pub struct RecoveryReport {
	pub cylinders: usize,
	pub spheres: usize,
	pub cones: usize,
	pub tori: usize,
	pub planes: usize,
	pub faces_before: usize,
	pub faces_after: usize,
	pub max_fit_residual: f64,
}

/// Dihedral bend (radians) below which two facets count as coplanar and are
/// NOT joined: exact planes are `coalesce_coplanar` territory, and joining
/// them here would let a flat wall masquerade as a huge-radius quadric.
const BEND_MIN: f64 = 1e-4;

/// Dihedral bend (radians) above which an edge is a feature crease and never
/// crossed (≈ 34°; a hexagonal prism's 60° corners stay corners, mesher-scale
/// smooth curvature of a few degrees per facet passes).
const BEND_MAX: f64 = 0.6;

/// Minimum facets for a region to be considered — a quadric fit on fewer is
/// noise.
const MIN_GROUP: usize = 8;

/// Minimum angular spread of a region's facet normals (radians): a
/// nearly-flat region (spread below this) is left planar rather than carried
/// by an enormous-radius quadric that happens to pass within `tol`.
const MIN_NORMAL_SPREAD: f64 = 0.15;

/// Fitted radii larger than this multiple of the region's own extent are
/// rejected as "actually a plane".
const RADIUS_SANITY: f64 = 1e3;

/// Angular span (radians) of one merged single-curved chart sector: a half
/// wrap. Two half-wrap sectors cover a full cylinder/cone — a single-loop face
/// cannot be a full periodic wrap (its boundary would be two disjoint rims),
/// and π keeps each sector's ring safely inside the injective domain of its
/// unwrap chart (< 2π). Fidelity comes from interior-refined tessellation, not
/// from this span (see the module doc), so the old 0.11-rad tessellation
/// budget applies only to the [`MergePolicy::LegacySectors`] fallback.
const CHART_SPAN: f64 = std::f64::consts::PI;

/// The legacy single-curved sector span budget (radians) — the honest limit of
/// face-count collapse through the *boundary-only* tessellation contract
/// (relative volume error of a sector ring ≈ span²/6 ≈ 0.2% at 0.11 rad).
/// Used only by the [`MergePolicy::LegacySectors`] fallback rung.
const LEGACY_SECTOR_SPAN_MAX: f64 = 0.11;

/// Maximum peel-and-refit rounds per (region, surface kind).
const PEEL_ROUNDS: usize = 24;

/// Accepted cone half-angle range (radians): below is a cylinder's job, above
/// is a near-plane cap.
const CONE_ANGLE_MIN: f64 = 0.01;
const CONE_ANGLE_MAX: f64 = 1.45;

/// Minimum angular span (radians, ≈ 20°) an accepted CURVED region must
/// subtend about its own fitted axis/center. A near-flat patch can always be
/// carried by an enormous-radius quadric within `tol` (a dual-contour cap's
/// rim ring "fits" a 365 mm cylinder lying on its side — measured), but such a
/// carrier subtends only milliradians of its own surface; a real recovered
/// band subtends a substantial arc. Planes are exempt (they have no arc).
const MIN_ARC: f64 = 0.35;

/// Relative tessellated-volume drift above which the rebuilt solid is refused.
const DRIFT_MAX: f64 = 0.005;

// --- small dense linear algebra -----------------------------------------------

/// Unit eigenvector of the smallest eigenvalue of a symmetric 3×3 matrix, via
/// cyclic Jacobi rotations (deterministic: fixed sweep order, fixed rounds;
/// ties resolve to the lowest index). Zero matrix → `DVec3::Z`.
fn smallest_eigenvector(m: DMat3) -> DVec3 {
	let mut a = [[m.col(0).x, m.col(1).x, m.col(2).x], [m.col(0).y, m.col(1).y, m.col(2).y], [m.col(0).z, m.col(1).z, m.col(2).z]];
	let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
	let scale = a.iter().flatten().fold(0.0_f64, |s, &x| s.max(x.abs()));
	if scale <= 0.0 {
		return DVec3::Z;
	}
	for _ in 0..32 {
		let off = a[0][1] * a[0][1] + a[0][2] * a[0][2] + a[1][2] * a[1][2];
		if off < 1e-30 * scale * scale {
			break;
		}
		for &(p, q) in &[(0usize, 1usize), (0, 2), (1, 2)] {
			if a[p][q].abs() < 1e-300 {
				continue;
			}
			let theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
			let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
			let c = 1.0 / (t * t + 1.0).sqrt();
			let s = t * c;
			for row in a.iter_mut() {
				let (akp, akq) = (row[p], row[q]);
				row[p] = c * akp - s * akq;
				row[q] = s * akp + c * akq;
			}
			let (rp, rq) = (a[p], a[q]);
			a[p] = std::array::from_fn(|k| c * rp[k] - s * rq[k]);
			a[q] = std::array::from_fn(|k| s * rp[k] + c * rq[k]);
			for r in &mut v {
				let (vp, vq) = (r[p], r[q]);
				r[p] = c * vp - s * vq;
				r[q] = s * vp + c * vq;
			}
		}
	}
	let eig = [a[0][0], a[1][1], a[2][2]];
	let mut best = 0;
	for k in 1..3 {
		if eig[k] < eig[best] {
			best = k;
		}
	}
	DVec3::new(v[0][best], v[1][best], v[2][best]).normalize_or_zero()
}

/// Kåsa algebraic circle fit in 2-D: least squares of `x² + y² = D·x + E·y + F`
/// → `(center, radius)`. `None` for a degenerate (collinear / too few) set.
fn kasa_circle(pts: &[(f64, f64)]) -> Option<((f64, f64), f64)> {
	if pts.len() < 3 {
		return None;
	}
	let n = pts.len() as f64;
	let (mut sx, mut sy, mut sxx, mut sxy, mut syy) = (0.0, 0.0, 0.0, 0.0, 0.0);
	let (mut sxz, mut syz, mut sz) = (0.0, 0.0, 0.0);
	for &(x, y) in pts {
		let z = x * x + y * y;
		sx += x;
		sy += y;
		sxx += x * x;
		sxy += x * y;
		syy += y * y;
		sxz += x * z;
		syz += y * z;
		sz += z;
	}
	let m = DMat3::from_cols(DVec3::new(sxx, sxy, sx), DVec3::new(sxy, syy, sy), DVec3::new(sx, sy, n));
	let det = m.determinant();
	let scale = (sxx + syy).max(1.0);
	if det.abs() < 1e-12 * scale * scale {
		return None;
	}
	let sol = m.inverse() * DVec3::new(sxz, syz, sz);
	let (cx, cy) = (sol.x * 0.5, sol.y * 0.5);
	let r2 = sol.z + cx * cx + cy * cy;
	if !(r2.is_finite() && r2 > 0.0) {
		return None;
	}
	Some(((cx, cy), r2.sqrt()))
}

/// Diagonal extent of a sample point cloud (fit sanity scale).
fn extent(samples: &[(DVec3, DVec3)]) -> f64 {
	let mut lo = DVec3::splat(f64::INFINITY);
	let mut hi = DVec3::splat(f64::NEG_INFINITY);
	for &(p, _) in samples {
		lo = lo.min(p);
		hi = hi.max(p);
	}
	let d = hi - lo;
	if d.is_finite() {
		d.length().max(1e-9)
	} else {
		1e-9
	}
}

// --- public fitters (AI-callable; each sample is (surface point, unit normal)) --

/// Least-squares **plane** through `samples`: origin at the point centroid,
/// normal the smallest-eigenvalue direction of the point covariance, oriented
/// along the mean sample normal. The zero-curvature member of the recovery
/// family — what merges a mesher cap's near-flat rim ring that the exact-key
/// `coalesce_coplanar` cannot. Judge with [`fit_residual`].
pub fn fit_plane(samples: &[(DVec3, DVec3)]) -> Option<Surface> {
	if samples.len() < 3 {
		return None;
	}
	let n_inv = 1.0 / samples.len() as f64;
	let centroid = samples.iter().fold(DVec3::ZERO, |s, &(p, _)| s + p) * n_inv;
	let mut cov = DMat3::ZERO;
	for &(p, _) in samples {
		let d = p - centroid;
		cov += outer(d, d);
	}
	let mut normal = smallest_eigenvector(cov);
	if normal.length_squared() < 0.5 {
		return None;
	}
	let mean_n = samples.iter().fold(DVec3::ZERO, |s, &(_, n)| s + n);
	if normal.dot(mean_n) < 0.0 {
		normal = -normal;
	}
	Some(Surface::Plane { origin: centroid, normal })
}

/// Least-squares **cylinder** through `samples` (`(point, outward unit normal)`
/// pairs, e.g. facet vertices/centroids with their facet normals). Axis from
/// the facet-normal great-circle fit (every cylinder normal is ⊥ the axis, so
/// the axis is the smallest-eigenvalue direction of `Σ n·nᵀ`); center/radius
/// from a Kåsa circle fit of the points projected along the axis, radius
/// refined to the mean radial distance. Returns `None` for degenerate input or
/// a radius so large the data is effectively planar. Judge the fit with
/// [`fit_residual`] — this constructor does not gate.
pub fn fit_cylinder(samples: &[(DVec3, DVec3)]) -> Option<Surface> {
	if samples.len() < 6 {
		return None;
	}
	let mut m = DMat3::ZERO;
	for &(_, n) in samples {
		m += outer(n, n);
	}
	let axis = smallest_eigenvector(m);
	if axis.length_squared() < 0.5 {
		return None;
	}
	let (e1, e2) = perp_basis(axis);
	let pts2: Vec<(f64, f64)> = samples.iter().map(|&(p, _)| (p.dot(e1), p.dot(e2))).collect();
	let ((cx, cy), _) = kasa_circle(&pts2)?;
	let radius = pts2.iter().map(|&(x, y)| ((x - cx) * (x - cx) + (y - cy) * (y - cy)).sqrt()).sum::<f64>() / pts2.len() as f64;
	if !(radius.is_finite() && radius > 1e-9 && radius <= RADIUS_SANITY * extent(samples)) {
		return None;
	}
	Some(Surface::Cylinder { origin: e1 * cx + e2 * cy, axis, radius })
}

/// Least-squares **sphere** through `samples`: the center is the point closest
/// to every sample's normal LINE (`p + t·n`), the classic `Σ(I − n nᵀ)`
/// system; the radius is the mean center distance. `None` when the normal
/// lines are near-parallel (a flat region) or the radius fails sanity.
pub fn fit_sphere(samples: &[(DVec3, DVec3)]) -> Option<Surface> {
	if samples.len() < 6 {
		return None;
	}
	let n_inv = 1.0 / samples.len() as f64;
	let mut a = DMat3::ZERO;
	let mut b = DVec3::ZERO;
	for &(p, n) in samples {
		let proj = DMat3::IDENTITY - outer(n, n);
		a += proj;
		b += proj * p;
	}
	let a = a * n_inv;
	if a.determinant().abs() < 1e-6 {
		return None;
	}
	let center = a.inverse() * (b * n_inv);
	let radius = samples.iter().map(|&(p, _)| (p - center).length()).sum::<f64>() * n_inv;
	if !(center.is_finite() && radius.is_finite() && radius > 1e-9 && radius <= RADIUS_SANITY * extent(samples)) {
		return None;
	}
	Some(Surface::Sphere { center, radius })
}

/// Least-squares **cone** through `samples`. The axis DIRECTION is the normal
/// of the circle the unit normals trace on the direction sphere (smallest
/// eigenvalue of the normal covariance, signed so it points from apex into
/// the body per [`Surface::Cone`]); the axis POSITION comes from the
/// meridian-plane condition (every cone normal is coplanar with the axis —
/// the same linear system the torus fit uses); apex and half-angle from a
/// line fit `radial = tan α · (h − h_apex)` in (axial, radial) coordinates —
/// unbiased for on-surface vertex samples, unlike a tangent-plane apex solve
/// whose chord planes all sit one sagitta inside the true cone. `None` for
/// cylinder-like (near-parallel radial law) or out-of-range geometry.
pub fn fit_cone(samples: &[(DVec3, DVec3)]) -> Option<Surface> {
	if samples.len() < 6 {
		return None;
	}
	let n_inv = 1.0 / samples.len() as f64;
	let mean_n = samples.iter().fold(DVec3::ZERO, |s, &(_, n)| s + n) * n_inv;
	let mut cov = DMat3::ZERO;
	for &(_, n) in samples {
		let d = n - mean_n;
		cov += outer(d, d);
	}
	let d = smallest_eigenvector(cov);
	if d.length_squared() < 0.5 {
		return None;
	}
	let (e1, e2) = perp_basis(d);
	let (cx, cy) = axis_point_xy(samples, e1, e2)?;
	// Line fit ρ = s·h + b over (h, ρ) — |s| = tan α, apex where ρ = 0.
	let (mut sh, mut sr, mut shh, mut shr) = (0.0, 0.0, 0.0, 0.0);
	for &(p, _) in samples {
		let h = p.dot(d);
		let (px, py) = (p.dot(e1) - cx, p.dot(e2) - cy);
		let rho = (px * px + py * py).sqrt();
		sh += h;
		sr += rho;
		shh += h * h;
		shr += h * rho;
	}
	let n = samples.len() as f64;
	let det = n * shh - sh * sh;
	if det.abs() < 1e-12 * (1.0 + shh) * n {
		return None;
	}
	let slope = (n * shr - sh * sr) / det;
	let intercept = (sr * shh - sh * shr) / det;
	if slope.abs() < 1e-12 {
		return None; // constant radius: a cylinder, not a cone
	}
	// Apex where the radius law hits zero (a flip-independent point); the
	// surface axis then runs apex → body, i.e. the direction of growing ρ.
	let apex = e1 * cx + e2 * cy + d * (-intercept / slope);
	let axis = d * slope.signum();
	let half_angle = slope.abs().atan();
	let ahead = samples.iter().map(|&(p, _)| (p - apex).dot(axis)).sum::<f64>() * n_inv;
	if !(apex.is_finite() && (CONE_ANGLE_MIN..=CONE_ANGLE_MAX).contains(&half_angle) && ahead > 0.0) {
		return None;
	}
	Some(Surface::Cone { apex, axis, half_angle })
}

/// Axis-line position in the `(e1, e2)` plane from the meridian-plane
/// condition: every surface normal of a rotational surface is coplanar with
/// the axis, so `(p_xy − c_xy) × u = 0` for the unit in-plane normal
/// component `u` — a linear 2-D least-squares for `c_xy`. Shared by the cone
/// and torus fitters. `None` when the normals carry no in-plane component
/// (a flat cap) or the system is degenerate.
fn axis_point_xy(samples: &[(DVec3, DVec3)], e1: DVec3, e2: DVec3) -> Option<(f64, f64)> {
	let (mut m00, mut m01, mut m11, mut r0, mut r1) = (0.0, 0.0, 0.0, 0.0, 0.0);
	let mut used = 0usize;
	for &(p, n) in samples {
		let (nx, ny) = (n.dot(e1), n.dot(e2));
		let w = (nx * nx + ny * ny).sqrt();
		if w < 1e-9 {
			continue;
		}
		let (ux, uy) = (nx / w, ny / w);
		let (px, py) = (p.dot(e1), p.dot(e2));
		let k = uy * px - ux * py;
		m00 += uy * uy;
		m01 -= ux * uy;
		m11 += ux * ux;
		r0 += uy * k;
		r1 -= ux * k;
		used += 1;
	}
	if used < 6 {
		return None;
	}
	let det = m00 * m11 - m01 * m01;
	if det.abs() < 1e-9 * (m00 + m11).max(1.0).powi(2) {
		return None;
	}
	Some(((r0 * m11 - r1 * m01) / det, (m00 * r1 - m01 * r0) / det))
}

/// Least-squares **torus** through `samples`. Axis candidates: the mean facet
/// normal (a partial rim-fillet band — its ring components cancel over the
/// azimuth) and the smallest-eigenvalue direction of the point covariance (a
/// full donut — the ring plane); for each, the axis line is placed by the
/// meridian-plane condition (every torus normal is coplanar with the axis, a
/// linear 2-D system) and the tube by a Kåsa circle fit in `(radial, axial)`
/// coordinates. The lower-residual candidate wins. `None` when neither
/// candidate produces a sane `minor < major` tube.
pub fn fit_torus(samples: &[(DVec3, DVec3)]) -> Option<Surface> {
	if samples.len() < 8 {
		return None;
	}
	let n_inv = 1.0 / samples.len() as f64;
	let mean_n = samples.iter().fold(DVec3::ZERO, |s, &(_, n)| s + n) * n_inv;
	let mean_p = samples.iter().fold(DVec3::ZERO, |s, &(p, _)| s + p) * n_inv;
	let mut cov = DMat3::ZERO;
	for &(p, _) in samples {
		let d = p - mean_p;
		cov += outer(d, d);
	}
	let mut cands: Vec<DVec3> = Vec::new();
	if mean_n.length() > 1e-6 {
		cands.push(mean_n.normalize());
	}
	cands.push(smallest_eigenvector(cov));
	let mut best: Option<(Surface, f64)> = None;
	for d in cands {
		if d.length_squared() < 0.5 {
			continue;
		}
		let Some(surf) = fit_torus_about(samples, d) else { continue };
		let resid = fit_residual(&surf, &samples.iter().map(|&(p, _)| p).collect::<Vec<_>>());
		if best.as_ref().is_none_or(|(_, r)| resid < *r) {
			best = Some((surf, resid));
		}
	}
	best.map(|(s, _)| s)
}

/// Torus fit for a FIXED axis direction `d` (see [`fit_torus`]): axis line
/// position from the shared meridian-plane system, then the tube as a Kåsa
/// circle in (radial distance, axial height) coordinates.
fn fit_torus_about(samples: &[(DVec3, DVec3)], d: DVec3) -> Option<Surface> {
	let (e1, e2) = perp_basis(d);
	let (cx, cy) = axis_point_xy(samples, e1, e2)?;
	let rh: Vec<(f64, f64)> = samples
		.iter()
		.map(|&(p, _)| {
			let (px, py) = (p.dot(e1) - cx, p.dot(e2) - cy);
			((px * px + py * py).sqrt(), p.dot(d))
		})
		.collect();
	let ((major, z0), minor) = kasa_circle(&rh)?;
	let ext = extent(samples);
	if !(minor.is_finite() && major.is_finite() && minor > 1e-9 && minor < major && major <= RADIUS_SANITY * ext) {
		return None;
	}
	Some(Surface::Torus { center: e1 * cx + e2 * cy + d * z0, axis: d, major, minor })
}

/// Maximum unsigned distance of `probes` from `surface` — the fit residual
/// oracle used by [`recover_quadrics`]'s acceptance gate. Feed it vertices AND
/// facet edge midpoints/centroids to make chord sagitta count (that is what
/// rejects a hexagonal prism as a "cylinder": its vertices sit exactly on the
/// circumscribed cylinder but its flat-side midpoints do not).
pub fn fit_residual(surface: &Surface, probes: &[DVec3]) -> f64 {
	probes.iter().map(|&p| surface.unsigned_distance(p)).fold(0.0, f64::max)
}

fn outer(a: DVec3, b: DVec3) -> DMat3 {
	DMat3::from_cols(b * a.x, b * a.y, b * a.z).transpose()
}

// --- the finishing pass ---------------------------------------------------------

/// Per-facet cached geometry for grouping/fitting.
struct Facet {
	/// Outer-loop vertex ids.
	verts: Vec<u32>,
	/// Unit Newell normal of the outer loop.
	normal: DVec3,
	centroid: DVec3,
	/// Fit-acceptance probes: vertices + edge midpoints + centroid.
	probes: Vec<DVec3>,
}

/// The carrier kind accepted for a region (drives report counters + rebuild).
#[derive(Clone, Copy, PartialEq)]
enum Kind {
	Plane,
	Cylinder,
	Sphere,
	Cone,
	Torus,
}

/// Evaluation order: simplest-first for ties, and the deterministic peel order.
const KINDS: [Kind; 5] = [Kind::Plane, Kind::Cylinder, Kind::Sphere, Kind::Cone, Kind::Torus];

/// The rebuild's merge aggressiveness — tried in order, first gate-passing
/// rung wins (module doc: honest fallback cascade). The cascade is what lets a
/// coarsely faceted input, whose interior-refined merge would legitimately
/// drift past the volume gate relative to its own chord tessellation, degrade
/// to the weaker-but-safe legacy behavior instead of refusing outright.
#[derive(Clone, Copy, PartialEq, Debug)]
enum MergePolicy {
	/// Full chart merge: single-curved half-wrap sectors, sphere cubemap
	/// sextants, torus 4 × 4 quadrant grid (interior-refined tessellation).
	ChartMerge,
	/// Single-curved half-wrap sectors; doubly-curved retagged facet-by-facet.
	DoublyRetag,
	/// The pre-chart behavior: 0.11-rad budgeted sectors + doubly-curved retag
	/// (fidelity through the boundary-only tessellation contract).
	LegacySectors,
}

/// Angular span (radians) the given points (facet centroids for the
/// [`MIN_ARC`] guard, ring vertices for the tessellator's merged-face gate)
/// subtend about the carrier. Azimuth extent about the axis for
/// cylinder/cone/torus (2π − largest gap between sorted azimuths), twice the
/// worst deviation from the mean radial direction for a sphere (π when the
/// directions cancel — a closed region), and +∞ for a plane (exempt).
pub(crate) fn angular_span(surface: &Surface, centroids: &[DVec3]) -> f64 {
	use std::f64::consts::{PI, TAU};
	let azimuth_extent = |anchor: DVec3, axis: DVec3| -> f64 {
		let (e1, e2) = perp_basis(axis);
		let mut thetas: Vec<f64> = centroids
			.iter()
			.filter_map(|&c| {
				let d = c - anchor;
				let radial = d - axis * d.dot(axis);
				(radial.length_squared() > 1e-18).then(|| radial.dot(e2).atan2(radial.dot(e1)))
			})
			.collect();
		if thetas.len() < 2 {
			return 0.0;
		}
		thetas.sort_by(f64::total_cmp);
		let mut max_gap = TAU - (thetas[thetas.len() - 1] - thetas[0]);
		for w in thetas.windows(2) {
			max_gap = max_gap.max(w[1] - w[0]);
		}
		TAU - max_gap
	};
	match *surface {
		Surface::Plane { .. } => f64::INFINITY,
		Surface::Cylinder { origin, axis, .. } => azimuth_extent(origin, axis),
		Surface::Cone { apex, axis, .. } => azimuth_extent(apex, axis),
		Surface::Torus { center, axis, .. } => azimuth_extent(center, axis),
		Surface::Sphere { center, .. } => {
			let dirs: Vec<DVec3> = centroids.iter().map(|&c| (c - center).normalize_or_zero()).collect();
			let mean = dirs.iter().fold(DVec3::ZERO, |s, &d| s + d);
			if mean.length() < 1e-6 * dirs.len() as f64 {
				return PI; // directions cancel: the region closes around the center
			}
			let m = mean.normalize();
			2.0 * dirs.iter().map(|d| d.dot(m).clamp(-1.0, 1.0).acos()).fold(0.0, f64::max)
		}
	}
}

/// Recover analytic quadric surface carriers on a faceted solid — the reverse
/// bridge's v2 finishing pass (see the module doc for the full contract).
///
/// `tol` (model units, mm) is the fit acceptance band: every recovered
/// region's vertices, facet edge midpoints and centroids lie within `tol` of
/// the fitted surface. No vertex is moved. Single-curved regions merge into
/// span-budgeted sector faces (face count collapses); doubly-curved regions
/// are retagged facet-by-facet (carrier recovery without collapse — stated,
/// not hidden). Solids with nothing to recover (already-analytic builders, a
/// hexagonal prism at tight `tol`) return a structurally unchanged clone and a
/// zeroed report, so the pass is idempotent.
///
/// Refuses loudly (`Err` with the measured counts) when `tol` is invalid, the
/// rebuilt solid fails [`validate`], or its default-tessellation volume drifts
/// more than 0.5% from the input's.
///
/// Example: `recover_quadrics(&faceted_cylinder, 0.05)` → `(solid, report)`
/// with `report.cylinders == 1` and the lateral facets replaced by
/// `Surface::Cylinder` sector faces.
pub fn recover_quadrics(solid: &Solid, tol: f64) -> Result<(Solid, RecoveryReport), String> {
	if !(tol.is_finite() && tol > 0.0) {
		return Err(format!("recover_quadrics: tol must be positive and finite, got {tol}"));
	}
	let nf = solid.face_count();
	let no_op = || RecoveryReport { faces_before: nf, faces_after: nf, ..RecoveryReport::default() };
	if nf == 0 {
		return Ok((solid.clone(), no_op()));
	}

	// ---- per-facet geometry (planar-tagged candidates only) -----------------
	let facets: Vec<Option<Facet>> = solid
		.faces()
		.map(|f| {
			if !matches!(solid.face(f).surface, Surface::Plane { .. }) {
				return None;
			}
			let verts: Vec<u32> = solid.face_vertices(f).iter().map(|v| v.0).collect();
			if verts.len() < 3 {
				return None;
			}
			let poly: Vec<DVec3> = verts.iter().map(|&v| solid.position(crate::topo::VertexId(v))).collect();
			let normal = newell(&poly);
			if normal.length_squared() < 0.5 {
				return None;
			}
			let centroid = poly.iter().copied().sum::<DVec3>() / poly.len() as f64;
			let mut probes = poly.clone();
			for i in 0..poly.len() {
				probes.push((poly[i] + poly[(i + 1) % poly.len()]) * 0.5);
			}
			probes.push(centroid);
			Some(Facet { verts, normal, centroid, probes })
		})
		.collect();

	// ---- region growing: smooth-bend union-find over shared edges -----------
	let mut parent: Vec<u32> = (0..nf as u32).collect();
	for e in solid.edges() {
		let he = solid.half_edge(solid.edge(e).half_edge);
		let Some(twin) = he.twin else { continue };
		let (f1, f2) = (he.face, solid.half_edge(twin).face);
		if f1 == f2 {
			continue;
		}
		let (Some(a), Some(b)) = (&facets[f1.0 as usize], &facets[f2.0 as usize]) else { continue };
		let bend = a.normal.dot(b.normal).clamp(-1.0, 1.0).acos();
		if (BEND_MIN..=BEND_MAX).contains(&bend) {
			let (r1, r2) = (find(&mut parent, f1.0), find(&mut parent, f2.0));
			parent[r1 as usize] = r2;
		}
	}
	let mut groups: std::collections::HashMap<u32, Vec<u32>> = std::collections::HashMap::new();
	for f in 0..nf as u32 {
		if facets[f as usize].is_some() {
			groups.entry(find(&mut parent, f)).or_default().push(f);
		}
	}
	let mut group_list: Vec<Vec<u32>> = groups.into_values().filter(|g| g.len() >= MIN_GROUP).collect();
	for g in group_list.iter_mut() {
		g.sort_unstable();
	}
	group_list.sort_by_key(|g| g[0]);

	// ---- carrier fitting: best full-group fit, then a peeling cascade --------
	struct Region {
		faces: Vec<u32>,
		surface: Surface,
		kind: Kind,
		vertex_residual: f64,
	}
	let facet_of = |f: u32| facets[f as usize].as_ref().unwrap();
	let collect_samples = |active: &[u32]| -> Vec<(DVec3, DVec3)> {
		let mut samples: Vec<(DVec3, DVec3)> = Vec::new();
		for &f in active {
			let fc = facet_of(f);
			samples.push((fc.centroid, fc.normal));
			for &v in &fc.verts {
				samples.push((solid.position(crate::topo::VertexId(v)), fc.normal));
			}
		}
		samples
	};
	let fit_kind = |kind: Kind, samples: &[(DVec3, DVec3)]| -> Option<Surface> {
		match kind {
			Kind::Plane => fit_plane(samples),
			Kind::Cylinder => fit_cylinder(samples),
			Kind::Sphere => fit_sphere(samples),
			Kind::Cone => fit_cone(samples),
			Kind::Torus => fit_torus(samples),
		}
	};
	// A fitted carrier is admissible for `active` when the arc it subtends
	// passes the MIN_ARC guard (planes are exempt — see `angular_span`).
	let span_ok = |surface: &Surface, active: &[u32]| -> bool {
		let centroids: Vec<DVec3> = active.iter().map(|&f| facet_of(f).centroid).collect();
		angular_span(surface, &centroids) >= MIN_ARC
	};
	let vertex_residual_of = |surface: &Surface, active: &[u32]| -> f64 {
		active
			.iter()
			.flat_map(|&f| facet_of(f).verts.iter())
			.map(|&v| surface.unsigned_distance(solid.position(crate::topo::VertexId(v))))
			.fold(0.0, f64::max)
	};
	let mut regions: Vec<Region> = Vec::new();
	for group in &group_list {
		// Normal-spread guard for CURVED carriers: a nearly flat region may
		// only be recovered as a tolerant plane, never as a huge-radius quadric.
		let mean_n = group.iter().fold(DVec3::ZERO, |s, &f| s + facet_of(f).normal);
		let spread = if mean_n.length() < 1e-9 {
			std::f64::consts::PI // normals cancel: a closed curved region (sphere/torus)
		} else {
			let m = mean_n.normalize();
			2.0 * group.iter().map(|&f| facet_of(f).normal.dot(m).clamp(-1.0, 1.0).acos()).fold(0.0, f64::max)
		};
		let kind_allowed = |kind: Kind| kind == Kind::Plane || spread >= MIN_NORMAL_SPREAD;

		// Phase 1 — whole-group fits, every kind: lowest residual within `tol`
		// wins (ties break toward KINDS order). This is what keeps a peeling
		// cylinder from "stealing" a thin equator band of a genuine torus: the
		// full-group torus fit passes and is compared before any peeling.
		let full_samples = collect_samples(group);
		let mut accepted: Option<Region> = None;
		let mut best_full: Option<(Kind, Surface, f64)> = None;
		for kind in KINDS {
			if !kind_allowed(kind) {
				continue;
			}
			let Some(surface) = fit_kind(kind, &full_samples) else { continue };
			let worst = group.iter().map(|&f| fit_residual(&surface, &facet_of(f).probes)).fold(0.0, f64::max);
			if worst <= tol && span_ok(&surface, group) && best_full.as_ref().is_none_or(|&(_, _, w)| worst < w) {
				best_full = Some((kind, surface, worst));
			}
		}
		if let Some((kind, surface, _)) = best_full {
			let vertex_residual = vertex_residual_of(&surface, group);
			accepted = Some(Region { faces: group.clone(), surface, kind, vertex_residual });
		}

		// Phase 2 — no whole-group carrier: peel outlier facets (a crease
		// chamfer ring welded to a clean cap or wall) and refit, for EVERY
		// kind; the candidate that explains the MOST facets wins (ties break
		// toward KINDS order). Selecting by explained coverage is what stops
		// a cylinder from claiming a thin full-ring band of a cone whose rim
		// rows spoiled the whole-group cone fit: the peeled cone keeps almost
		// everything, the peeled cylinder keeps only its band. Peeled facets
		// stay honest planar facets; they are not re-admitted.
		if accepted.is_none() {
			'kinds: for kind in KINDS {
				if !kind_allowed(kind) {
					continue;
				}
				let mut active: Vec<u32> = group.clone();
				for _round in 0..PEEL_ROUNDS {
					if active.len() < MIN_GROUP || accepted.as_ref().is_some_and(|a| a.faces.len() >= active.len()) {
						continue 'kinds; // cannot beat the current best coverage
					}
					let samples = collect_samples(&active);
					let Some(surface) = fit_kind(kind, &samples) else { continue 'kinds };
					let per_facet: Vec<f64> = active.iter().map(|&f| fit_residual(&surface, &facet_of(f).probes)).collect();
					let worst = per_facet.iter().copied().fold(0.0, f64::max);
					if worst <= tol {
						if !span_ok(&surface, &active) {
							continue 'kinds; // e.g. a milliradian arc of a 365 mm "cylinder"
						}
						let vertex_residual = vertex_residual_of(&surface, &active);
						accepted = Some(Region { faces: active, surface, kind, vertex_residual });
						continue 'kinds;
					}
					let kept: Vec<u32> = active.iter().zip(&per_facet).filter(|&(_, &r)| r <= tol).map(|(&f, _)| f).collect();
					if kept.len() == active.len() {
						continue 'kinds; // defensive: worst > tol implies at least one peel
					}
					active = kept;
				}
			}
		}
		if let Some(r) = accepted {
			regions.push(r);
		}
	}
	if regions.is_empty() {
		return Ok((solid.clone(), no_op()));
	}

	// ---- rebuild + gates, per merge policy (first policy that passes wins) ---
	// Untouched faces keep patch id = MAX (re-emitted verbatim). Tolerant-plane
	// regions merge whole (flat chords are exact — no volume budget needed, and
	// planar faces may carry hole loops); curved regions merge per chart bin
	// (see the module doc's chart policy) or retag, per `MergePolicy`.
	const NONE: u32 = u32::MAX;
	/// Singleton marker: the face is retagged, never merged.
	const SINGLETON: u64 = u64::MAX - 1;
	let rebuild = |policy: MergePolicy| -> Solid {
		let mut patch_of: Vec<u32> = vec![NONE; nf];
		let mut patch_surface: Vec<Option<Surface>> = vec![None; nf]; // per FACE: fitted carrier (retag or merge)
		let mut patch_multiloop: Vec<bool> = vec![false; nf]; // per FACE: may the merged face keep hole loops?
		let mut bin_of: Vec<u64> = vec![u64::MAX; nf];
		for (ri, region) in regions.iter().enumerate() {
			let sectors = match region.surface {
				Surface::Cylinder { origin, axis, radius } => Some((origin, axis, radius)),
				Surface::Cone { apex, axis, .. } => {
					let r_max = region
						.faces
						.iter()
						.flat_map(|&f| facets[f as usize].as_ref().unwrap().verts.iter())
						.map(|&v| {
							let d = solid.position(crate::topo::VertexId(v)) - apex;
							(d - axis * d.dot(axis)).length()
						})
						.fold(0.0, f64::max);
					Some((apex, axis, r_max.max(1e-9)))
				}
				_ => None,
			};
			match (region.kind, sectors) {
				(Kind::Plane, _) => {
					// One bin: the whole region merges into one (possibly holed) face.
					for &f in &region.faces {
						bin_of[f as usize] = (ri as u64) << 32;
						patch_surface[f as usize] = Some(region.surface);
						patch_multiloop[f as usize] = true;
					}
				}
				(_, Some((anchor, axis, r_eff))) => {
					// Single-curved azimuth bins: two half-wrap chart sectors, or the
					// legacy budget (never wider than the span whose chord sag would
					// exceed `tol`) on the fallback rung.
					let span = if policy == MergePolicy::LegacySectors {
						let span_tol = if tol < r_eff { 2.0 * (1.0 - tol / r_eff).clamp(-1.0, 1.0).acos() } else { std::f64::consts::TAU };
						span_tol.clamp(1e-6, LEGACY_SECTOR_SPAN_MAX)
					} else {
						CHART_SPAN
					};
					let k = (std::f64::consts::TAU / span).ceil().max(1.0) as u64;
					let (e1, e2) = perp_basis(axis);
					for &f in &region.faces {
						let c = facets[f as usize].as_ref().unwrap().centroid - anchor;
						let theta = c.dot(e2).atan2(c.dot(e1)); // (−π, π]
						let bin = (((theta + std::f64::consts::PI) / (std::f64::consts::TAU / k as f64)) as u64).min(k - 1);
						bin_of[f as usize] = (ri as u64) << 32 | bin;
						patch_surface[f as usize] = Some(region.surface);
					}
				}
				_ => {
					// Doubly-curved: chart bins on the full-merge rung (sphere →
					// cubemap sextants by the dominant axis of the center→centroid
					// direction; torus → 4 × 4 azimuth × tube-angle quadrants),
					// singleton retag on the fallback rungs.
					let chart_bin: Option<Box<dyn Fn(DVec3) -> u64>> = match (policy, region.surface) {
						(MergePolicy::ChartMerge, Surface::Sphere { center, .. }) => Some(Box::new(move |c: DVec3| {
							let d = c - center;
							let a = [d.x.abs(), d.y.abs(), d.z.abs()];
							let k = if a[0] >= a[1] && a[0] >= a[2] {
								0
							} else if a[1] >= a[2] {
								1
							} else {
								2
							};
							(k as u64) * 2 + u64::from([d.x, d.y, d.z][k] < 0.0)
						})),
						(MergePolicy::ChartMerge, Surface::Torus { center, axis, major, .. }) => {
							let (e1, e2) = perp_basis(axis);
							Some(Box::new(move |c: DVec3| {
								let d = c - center;
								let h = d.dot(axis);
								let radial = d - axis * h;
								let theta = radial.dot(e2).atan2(radial.dot(e1)); // (−π, π]
								let psi = h.atan2(radial.length() - major); // (−π, π]
								let quad = |t: f64| (((t + std::f64::consts::PI) / std::f64::consts::FRAC_PI_2) as u64).min(3);
								quad(theta) * 4 + quad(psi)
							}))
						}
						_ => None,
					};
					for &f in &region.faces {
						bin_of[f as usize] = match &chart_bin {
							Some(bin) => (ri as u64) << 32 | bin(facets[f as usize].as_ref().unwrap().centroid),
							None => SINGLETON,
						};
						patch_surface[f as usize] = Some(region.surface);
					}
				}
			}
		}
		// Connected components within equal bins → patch ids.
		let mut patch_parent: Vec<u32> = (0..nf as u32).collect();
		for e in solid.edges() {
			let he = solid.half_edge(solid.edge(e).half_edge);
			let Some(twin) = he.twin else { continue };
			let (f1, f2) = (he.face, solid.half_edge(twin).face);
			if f1 == f2 {
				continue;
			}
			let (b1, b2) = (bin_of[f1.0 as usize], bin_of[f2.0 as usize]);
			if b1 != u64::MAX && b1 != SINGLETON && b1 == b2 {
				let (r1, r2) = (find(&mut patch_parent, f1.0), find(&mut patch_parent, f2.0));
				patch_parent[r1 as usize] = r2;
			}
		}
		for f in 0..nf as u32 {
			patch_of[f as usize] = if bin_of[f as usize] == u64::MAX { NONE } else { find(&mut patch_parent, f) };
		}

		// ---- emit faces (deterministic: patches ordered by smallest member id),
		// carrying provenance: verbatim/retag faces keep their FaceName exactly, a
		// merged face inherits the lexicographically-least constituent name (the
		// policy documented on `FaceName`).
		let positions: Vec<DVec3> = (0..solid.vertex_count() as u32).map(|i| solid.position(crate::topo::VertexId(i))).collect();
		let mut patch_members: std::collections::HashMap<u32, Vec<u32>> = std::collections::HashMap::new();
		let mut order: Vec<(u32, Option<u32>)> = Vec::new(); // (first face id, patch root or None=verbatim single)
		for f in 0..nf as u32 {
			match patch_of[f as usize] {
				NONE => order.push((f, None)),
				root => {
					let members = patch_members.entry(root).or_default();
					if members.is_empty() {
						order.push((f, Some(root)));
					}
					members.push(f);
				}
			}
		}
		order.sort_by_key(|&(first, _)| first);

		let verbatim_loops = |f: FaceId| -> Vec<Vec<u32>> {
			let face = solid.face(f);
			std::iter::once(face.outer)
				.chain(face.inner.iter().copied())
				.map(|lp| solid.loop_half_edges(lp).iter().map(|&he| solid.half_edge(he).origin.0).collect())
				.collect()
		};
		let mut faces_out: Vec<FaceLoops> = Vec::new();
		let mut names_out: Vec<Option<FaceName>> = Vec::new();
		let merged_name = |members: &[u32]| -> Option<FaceName> {
			members.iter().map(|&f| solid.face_name(FaceId(f))).collect::<Option<Vec<_>>>()?.into_iter().min()
		};
		for (first, root) in order {
			let Some(root) = root else {
				faces_out.push(FaceLoops { loops: verbatim_loops(FaceId(first)), surface: solid.face(FaceId(first)).surface });
				names_out.push(solid.face_name(FaceId(first)));
				continue;
			};
			let members = &patch_members[&root];
			let fitted = patch_surface[first as usize].expect("patched face carries its fitted surface");
			if members.len() == 1 {
				faces_out.push(FaceLoops { loops: verbatim_loops(FaceId(first)), surface: fitted });
				names_out.push(solid.face_name(FaceId(first)));
				continue;
			}
			// Region-boundary half-edge walk (the coalesce_coplanar algorithm, keyed
			// by patch membership). Merged CURVED faces must come out as ONE loop —
			// a curved face with holes is neither tessellatable from its boundary
			// nor STEP-importable; merged PLANE faces may keep hole loops (outer =
			// largest projected area, exactly coalesce_coplanar's rule). Anything
			// else falls back to verbatim retag.
			let multiloop = patch_multiloop[first as usize];
			// A merged loop that visits the same vertex twice is PINCHED: the patch
			// is connected through a neck (a jagged mesher region binned by chart
			// quadrants does this), so the "boundary" traverses an interior edge
			// there and back. Such a face triangulates over that edge twice and the
			// welded mesh reads it as non-manifold — measured on the recovered
			// implicit torus, 11 four-triangle edges. Refuse the merge for that
			// patch and re-emit its facets verbatim (still carrier-retagged).
			let pinched = |loops: &[Vec<u32>]| {
				loops.iter().any(|lp| {
					let mut seen: BTreeSet<u32> = BTreeSet::new();
					lp.iter().any(|v| !seen.insert(*v))
				})
			};
			match walk_patch_boundary(solid, members, &patch_of, root).filter(|l| !pinched(l)) {
				Some(loops) if loops.len() == 1 => {
					faces_out.push(FaceLoops { loops, surface: fitted });
					names_out.push(merged_name(members));
				}
				Some(mut loops) if multiloop => {
					let n = match fitted {
						Surface::Plane { normal, .. } => normal,
						_ => unreachable!("multiloop merges are plane regions only"),
					};
					let area = |lp: &Vec<u32>| -> f64 {
						let mut a = DVec3::ZERO;
						for i in 0..lp.len() {
							let p = positions[lp[i] as usize];
							let q = positions[lp[(i + 1) % lp.len()] as usize];
							a += p.cross(q);
						}
						(a.dot(n) * 0.5).abs()
					};
					let outer_ix = (0..loops.len()).max_by(|&i, &j| area(&loops[i]).total_cmp(&area(&loops[j]))).unwrap();
					loops.swap(0, outer_ix);
					faces_out.push(FaceLoops { loops, surface: fitted });
					names_out.push(merged_name(members));
				}
				_ => {
					for &f in members {
						faces_out.push(FaceLoops { loops: verbatim_loops(FaceId(f)), surface: fitted });
						names_out.push(solid.face_name(FaceId(f)));
					}
				}
			}
		}

		// ---- compact to referenced vertices (coalesce lesson: phantom entries
		// corrupt χ) and rebuild --------------------------------------------------
		let mut remap: Vec<u32> = vec![u32::MAX; positions.len()];
		let mut compact: Vec<DVec3> = Vec::new();
		for fl in &mut faces_out {
			for lp in &mut fl.loops {
				for ix in lp.iter_mut() {
					if remap[*ix as usize] == u32::MAX {
						remap[*ix as usize] = compact.len() as u32;
						compact.push(positions[*ix as usize]);
					}
					*ix = remap[*ix as usize];
				}
			}
		}
		let mut out = Solid::from_faces_multiloop(compact, faces_out);
		// Provenance carry (all-or-nothing, like heal): every emitted face must
		// have a name and the input must have carried provenance at all.
		if !solid.provenance.is_empty() {
			if let Some(names) = names_out.into_iter().collect::<Option<Vec<FaceName>>>() {
				if names.len() == out.face_count() {
					out.set_provenance(names);
				}
			}
		}
		// Analytic edge curves survive when both endpoints survive (heal's rule).
		for e in solid.edges() {
			if let Some(c) = solid.edge_curve(e) {
				let he = solid.half_edge(solid.edge(e).half_edge);
				let a = remap[he.origin.0 as usize];
				let b = remap[solid.half_edge(he.next).origin.0 as usize];
				if a != u32::MAX && b != u32::MAX && a != b {
					out.set_edge_curve(crate::topo::VertexId(a), crate::topo::VertexId(b), c);
				}
			}
		}
		out
	};

	// ---- refusal gates, per policy (module doc: honest fallback cascade) -----
	let report_for = |out: &Solid| RecoveryReport {
		cylinders: regions.iter().filter(|r| r.kind == Kind::Cylinder).count(),
		spheres: regions.iter().filter(|r| r.kind == Kind::Sphere).count(),
		cones: regions.iter().filter(|r| r.kind == Kind::Cone).count(),
		tori: regions.iter().filter(|r| r.kind == Kind::Torus).count(),
		planes: regions.iter().filter(|r| r.kind == Kind::Plane).count(),
		faces_before: nf,
		faces_after: out.face_count(),
		max_fit_residual: regions.iter().map(|r| r.vertex_residual).fold(0.0, f64::max),
	};
	let v_in = volume(solid);
	let mut last_err = String::new();
	for policy in [MergePolicy::ChartMerge, MergePolicy::DoublyRetag, MergePolicy::LegacySectors] {
		let out = rebuild(policy);
		let report = report_for(&out);
		let v = validate(&out);
		if !v.is_valid() {
			last_err = format!(
				"recover_quadrics: rebuilt solid failed validation ({v:?}) under {policy:?} after recovering {} region(s) \
				 ({} cyl / {} sph / {} cone / {} torus / {} plane; faces {} → {}, max fit residual {:.6} mm) — refusing to hand back broken topology",
				regions.len(),
				report.cylinders,
				report.spheres,
				report.cones,
				report.tori,
				report.planes,
				report.faces_before,
				report.faces_after,
				report.max_fit_residual
			);
			continue;
		}
		// One tessellation answers both gates. WATERTIGHTNESS is a gate in its own
		// right: a merged chart face whose neighbour ear-clips an identical chord
		// (or whose jagged region binning leaves a doubled edge) yields a closed,
		// valid B-rep whose default mesh is NOT edge-closed — measured on a
		// mesher-derived torus, 11 four-triangle edges. Such a rebuild is refused
		// and the cascade drops to the next rung rather than handing back a solid
		// that cannot be meshed for print or export.
		let mesh = crate::tessellate::tessellate_default(&out);
		if !mesh.is_watertight() {
			last_err = format!(
				"recover_quadrics: the rebuilt solid's default tessellation is not watertight under {policy:?} \
				 ({} non-manifold edges over {} triangles; faces {} → {}) — refusing a merge whose faces cannot be meshed",
				mesh.non_manifold_edge_count(),
				mesh.indices.len() / 3,
				report.faces_before,
				report.faces_after
			);
			continue;
		}
		let v_out = mesh.signed_volume();
		let drift = (v_out - v_in).abs() / v_in.abs().max(1e-9);
		if drift > DRIFT_MAX {
			last_err = format!(
				"recover_quadrics: tessellated volume drifted {:.4}% under {policy:?} (input {v_in:.6} mm³ → rebuilt {v_out:.6} mm³, bar {:.2}%) \
				 across {} recovered region(s) (faces {} → {}) — refusing silently altered geometry",
				drift * 100.0,
				DRIFT_MAX * 100.0,
				regions.len(),
				report.faces_before,
				report.faces_after
			);
			continue;
		}
		return Ok((out, report));
	}
	Err(format!("{last_err} (every merge-policy rung failed — see the module doc's fallback cascade)"))
}

/// Union-find `find` with path halving.
fn find(p: &mut [u32], mut i: u32) -> u32 {
	while p[i as usize] != i {
		p[i as usize] = p[p[i as usize] as usize];
		i = p[i as usize];
	}
	i
}

/// Unit Newell normal of a polygon (winding-following).
fn newell(poly: &[DVec3]) -> DVec3 {
	let mut n = DVec3::ZERO;
	let len = poly.len();
	for i in 0..len {
		let (c, d) = (poly[i], poly[(i + 1) % len]);
		n.x += (c.y - d.y) * (c.z + d.z);
		n.y += (c.z - d.z) * (c.x + d.x);
		n.z += (c.x - d.x) * (c.y + d.y);
	}
	n.normalize_or_zero()
}

/// The region-boundary half-edge walk of `coalesce_coplanar`, generalized to an
/// arbitrary patch (faces whose `patch_of` equals `root`): boundary half-edges
/// are those whose twin lies outside the patch; chains follow `next`, hopping
/// `twin.next` across interior edges. Returns `None` when a chain fails to
/// close (defensive; the caller falls back to verbatim re-emission).
fn walk_patch_boundary(solid: &Solid, members: &[u32], patch_of: &[u32], root: u32) -> Option<Vec<Vec<u32>>> {
	let in_patch = |f: FaceId| patch_of[f.0 as usize] == root;
	let is_boundary = |he_id: crate::topo::HalfEdgeId| -> bool {
		match solid.half_edge(he_id).twin {
			Some(t) => !in_patch(solid.half_edge(t).face),
			None => true,
		}
	};
	let mut boundary: BTreeSet<u32> = BTreeSet::new();
	for &f in members {
		let face = solid.face(FaceId(f));
		for lp in std::iter::once(face.outer).chain(face.inner.iter().copied()) {
			for &he_id in &solid.loop_half_edges(lp) {
				if is_boundary(he_id) {
					boundary.insert(he_id.0);
				}
			}
		}
	}
	let next_boundary = |he_id: crate::topo::HalfEdgeId| -> Option<crate::topo::HalfEdgeId> {
		let mut n = solid.half_edge(he_id).next;
		for _ in 0..solid.half_edge_count() {
			if is_boundary(n) {
				return Some(n);
			}
			n = solid.half_edge(solid.half_edge(n).twin?).next;
		}
		None
	};
	let mut loops: Vec<Vec<u32>> = Vec::new();
	while let Some(&start_raw) = boundary.iter().next() {
		let start = crate::topo::HalfEdgeId(start_raw);
		let mut lp: Vec<u32> = Vec::new();
		let mut cur = start;
		loop {
			boundary.remove(&cur.0);
			lp.push(solid.half_edge(cur).origin.0);
			match next_boundary(cur) {
				Some(nx) if nx == start => break,
				Some(nx) => cur = nx,
				None => return None,
			}
		}
		if lp.len() < 3 {
			return None;
		}
		loops.push(lp);
	}
	if loops.is_empty() {
		None
	} else {
		Some(loops)
	}
}
