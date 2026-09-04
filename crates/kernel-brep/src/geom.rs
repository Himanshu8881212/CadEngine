// Copyright (c) LMCAD. Licensed under the MIT License.

//! Analytic geometry for the exact B-rep half — closed-form surfaces and curves
//! in `f64` (per the spec: precision on the B-rep side).
//!
//! Each [`Surface`] supplies the outward unit normal at a surface point and the
//! unsigned distance to the surface; each [`Curve`] supplies positions and
//! tangents. NURBS is deliberately out of scope (the largest complexity sink).

use kernel_core::math::{DAffine3, DVec2, DVec3};

/// An orthonormal basis `(u, v)` spanning the plane perpendicular to unit `axis`.
pub fn perp_basis(axis: DVec3) -> (DVec3, DVec3) {
	let a = if axis.x.abs() < 0.9 { DVec3::X } else { DVec3::Y };
	let u = (a - axis * a.dot(axis)).normalize();
	(u, axis.cross(u))
}

/// Integrate `f` over `[t0, t1]` with composite 5-point Gauss–Legendre quadrature (32
/// panels). Exact for polynomials up to degree 9 per panel and machine-exact for the
/// smooth analytic conic speeds it integrates. Used by [`Curve::length`].
fn gauss_legendre_5<F: Fn(f64) -> f64>(f: F, t0: f64, t1: f64) -> f64 {
	const X: [f64; 5] = [-0.906_179_845_938_664, -0.538_469_310_105_683, 0.0, 0.538_469_310_105_683, 0.906_179_845_938_664];
	const W: [f64; 5] = [0.236_926_885_056_189, 0.478_628_670_499_366, 0.568_888_888_888_889, 0.478_628_670_499_366, 0.236_926_885_056_189];
	const PANELS: usize = 32;
	let h = (t1 - t0) / PANELS as f64;
	let mut sum = 0.0;
	for p in 0..PANELS {
		let mid = t0 + h * (p as f64 + 0.5);
		for k in 0..5 {
			sum += W[k] * f(mid + 0.5 * h * X[k]);
		}
	}
	sum * 0.5 * h
}

/// A closed-form analytic surface. All direction fields are expected unit length.
#[derive(Clone, Copy, Debug)]
pub enum Surface {
	Plane {
		origin: DVec3,
		normal: DVec3,
	},
	Cylinder {
		origin: DVec3,
		axis: DVec3,
		radius: f64,
	},
	Sphere {
		center: DVec3,
		radius: f64,
	},
	/// Cone with `axis` pointing from `apex` into the body; `half_angle` in radians.
	Cone {
		apex: DVec3,
		axis: DVec3,
		half_angle: f64,
	},
	Torus {
		center: DVec3,
		axis: DVec3,
		major: f64,
		minor: f64,
	},
}

impl Surface {
	/// Outward unit normal at point `p` (assumed on the surface).
	pub fn normal_at(&self, p: DVec3) -> DVec3 {
		match *self {
			Surface::Plane { normal, .. } => normal,
			Surface::Cylinder { origin, axis, .. } => {
				let d = p - origin;
				let radial = d - axis * d.dot(axis);
				radial.normalize_or_zero()
			}
			Surface::Sphere { center, .. } => (p - center).normalize_or_zero(),
			Surface::Cone { apex, axis, half_angle } => {
				// Outward normal of a cone tilts off the radial direction by the
				// half angle toward the axis.
				let d = p - apex;
				let h = d.dot(axis);
				let radial = (d - axis * h).normalize_or_zero();
				(radial * half_angle.cos() - axis * half_angle.sin()).normalize_or_zero()
			}
			Surface::Torus { center, axis, major, .. } => {
				let d = p - center;
				let h = d.dot(axis);
				let radial = d - axis * h;
				let ring_dir = radial.normalize_or_zero();
				let tube_center = center + ring_dir * major;
				(p - tube_center).normalize_or_zero()
			}
		}
	}

	/// Unsigned Euclidean distance from `p` to the surface.
	pub fn unsigned_distance(&self, p: DVec3) -> f64 {
		match *self {
			Surface::Plane { origin, normal } => (p - origin).dot(normal).abs(),
			Surface::Cylinder { origin, axis, radius } => {
				let d = p - origin;
				let radial = (d - axis * d.dot(axis)).length();
				(radial - radius).abs()
			}
			Surface::Sphere { center, radius } => ((p - center).length() - radius).abs(),
			Surface::Cone { apex, axis, half_angle } => {
				let d = p - apex;
				let h = d.dot(axis);
				// Behind the apex (h < 0) the nearest surface point is the apex itself;
				// the generating-line formula would otherwise underestimate the distance.
				if h < 0.0 {
					d.length()
				} else {
					// Distance to the cone's generating line in the axial/radial plane.
					let radial = (d - axis * h).length();
					(radial * half_angle.cos() - h * half_angle.sin()).abs()
				}
			}
			Surface::Torus { center, axis, major, minor } => {
				let d = p - center;
				let h = d.dot(axis);
				let radial = (d - axis * h).length();
				(DVec2::new(radial - major, h).length() - minor).abs()
			}
		}
	}

	/// Signed implicit field value: negative inside, positive outside, zero on the
	/// surface. Unlike a foot-point projection this stays correct on the medial axis
	/// (the cylinder axis, a sphere's centre), so it is a robust [`crate::ssi`] field.
	pub fn signed_value(&self, p: DVec3) -> f64 {
		match *self {
			Surface::Plane { origin, normal } => (p - origin).dot(normal),
			Surface::Cylinder { origin, axis, radius } => {
				let d = p - origin;
				(d - axis * d.dot(axis)).length() - radius
			}
			Surface::Sphere { center, radius } => (p - center).length() - radius,
			Surface::Cone { apex, axis, half_angle } => {
				let d = p - apex;
				let h = d.dot(axis);
				if h < 0.0 {
					d.length() // behind the apex: outside the single nappe
				} else {
					let radial = (d - axis * h).length();
					radial * half_angle.cos() - h * half_angle.sin()
				}
			}
			Surface::Torus { center, axis, major, minor } => {
				let d = p - center;
				let h = d.dot(axis);
				let radial = (d - axis * h).length();
				DVec2::new(radial - major, h).length() - minor
			}
		}
	}

	/// A point on the surface from its natural `(u, v)` parameters, where `u` is
	/// an angle (radians) for the curved surfaces. Used by constructors.
	pub fn point_at(&self, u: f64, v: f64) -> DVec3 {
		match *self {
			Surface::Plane { origin, normal } => {
				let (e1, e2) = perp_basis(normal);
				origin + e1 * u + e2 * v
			}
			Surface::Cylinder { origin, axis, radius } => {
				let (e1, e2) = perp_basis(axis);
				origin + (e1 * u.cos() + e2 * u.sin()) * radius + axis * v
			}
			Surface::Sphere { center, radius } => {
				// u = azimuth, v = polar angle from +axis (here world +Z basis).
				center + DVec3::new(v.sin() * u.cos(), v.sin() * u.sin(), v.cos()) * radius
			}
			Surface::Cone { apex, axis, half_angle } => {
				let (e1, e2) = perp_basis(axis);
				let r = v * half_angle.tan();
				apex + axis * v + (e1 * u.cos() + e2 * u.sin()) * r
			}
			Surface::Torus { center, axis, major, minor } => {
				let (e1, e2) = perp_basis(axis);
				let ring = e1 * u.cos() + e2 * u.sin();
				center + ring * (major + minor * v.cos()) + axis * (minor * v.sin())
			}
		}
	}

	/// Snap a near-surface point onto the surface (used to refine tessellation
	/// of curved faces so subdivided points land exactly on the analytic shape).
	pub fn project(&self, p: DVec3) -> DVec3 {
		match *self {
			Surface::Plane { origin, normal } => p - normal * (p - origin).dot(normal),
			Surface::Cylinder { origin, axis, radius } => {
				let d = p - origin;
				let h = d.dot(axis);
				let rn = (d - axis * h).normalize_or_zero();
				origin + axis * h + rn * radius
			}
			Surface::Sphere { center, radius } => center + (p - center).normalize_or_zero() * radius,
			Surface::Cone { apex, axis, half_angle } => {
				let d = p - apex;
				let h = d.dot(axis).max(0.0);
				let rn = (d - axis * h).normalize_or_zero();
				apex + axis * h + rn * (h * half_angle.tan())
			}
			Surface::Torus { center, axis, major, minor } => {
				let d = p - center;
				let h = d.dot(axis);
				let rn = (d - axis * h).normalize_or_zero();
				let tube_center = center + rn * major;
				tube_center + (p - tube_center).normalize_or_zero() * minor
			}
		}
	}

	/// Transform the surface by `m` (rigid + uniform scale).
	pub fn transformed(&self, m: DAffine3) -> Surface {
		let scale = m.matrix3.x_axis.length();
		let pt = |p: DVec3| m.transform_point3(p);
		let dir = |d: DVec3| m.transform_vector3(d).normalize_or_zero();
		match *self {
			Surface::Plane { origin, normal } => Surface::Plane { origin: pt(origin), normal: dir(normal) },
			Surface::Cylinder { origin, axis, radius } => Surface::Cylinder { origin: pt(origin), axis: dir(axis), radius: radius * scale },
			Surface::Sphere { center, radius } => Surface::Sphere { center: pt(center), radius: radius * scale },
			Surface::Cone { apex, axis, half_angle } => Surface::Cone { apex: pt(apex), axis: dir(axis), half_angle },
			Surface::Torus { center, axis, major, minor } => {
				Surface::Torus { center: pt(center), axis: dir(axis), major: major * scale, minor: minor * scale }
			}
		}
	}

	/// Whether two surfaces describe the **same geometric locus** within `tol`
	/// (model units on positions/radii; the same magnitude on direction cross
	/// products). Sign-insensitive where the locus is (plane normal, cylinder /
	/// torus axis); sign-sensitive for the single-nappe cone. Used by the boolean
	/// seam snapper to deduplicate the analytic surfaces meeting at a vertex —
	/// conservative by design: a false *negative* merely skips a snap, while a
	/// false positive could merge two genuinely different surfaces, so every
	/// parameter is compared.
	pub(crate) fn same_locus(&self, other: &Surface, tol: f64) -> bool {
		let dir_eq = |a: DVec3, b: DVec3| a.cross(b).length() < tol; // parallel (either sign)
		match (*self, *other) {
			(Surface::Plane { origin: o1, normal: n1 }, Surface::Plane { origin: o2, normal: n2 }) => {
				dir_eq(n1, n2) && (o2 - o1).dot(n1).abs() < tol
			}
			(Surface::Cylinder { origin: o1, axis: a1, radius: r1 }, Surface::Cylinder { origin: o2, axis: a2, radius: r2 }) => {
				let d = o2 - o1;
				dir_eq(a1, a2) && (r1 - r2).abs() < tol && (d - a1 * d.dot(a1)).length() < tol
			}
			(Surface::Sphere { center: c1, radius: r1 }, Surface::Sphere { center: c2, radius: r2 }) => {
				(c1 - c2).length() < tol && (r1 - r2).abs() < tol
			}
			(Surface::Cone { apex: p1, axis: a1, half_angle: h1 }, Surface::Cone { apex: p2, axis: a2, half_angle: h2 }) => {
				// Same nappe only: the axis must point the same way.
				(p1 - p2).length() < tol && a1.dot(a2) > 0.0 && dir_eq(a1, a2) && (h1 - h2).abs() < tol
			}
			(
				Surface::Torus { center: c1, axis: a1, major: m1, minor: n1 },
				Surface::Torus { center: c2, axis: a2, major: m2, minor: n2 },
			) => (c1 - c2).length() < tol && dir_eq(a1, a2) && (m1 - m2).abs() < tol && (n1 - n2).abs() < tol,
			_ => false,
		}
	}

	/// Exact analytic intersection of this surface with the plane through
	/// `plane_origin` with normal `plane_normal` (need not be unit-length).
	///
	/// Returns the closed-form conic section(s) — a [`Curve::Line`],
	/// [`Curve::Circle`], or [`Curve::Ellipse`] — with no meshing or numerical
	/// marching. An empty result means the surface and plane do not meet, or the
	/// case is not yet handled in closed form (a cone's parabola/hyperbola
	/// sections, and all torus sections).
	///
	/// ```
	/// use kernel_brep::{Curve, Surface};
	/// use kernel_brep::math::DVec3;
	/// // A radius-5 sphere cut by the plane z = 3 is a circle of radius √(5²−3²) = 4.
	/// let s = Surface::Sphere { center: DVec3::ZERO, radius: 5.0 };
	/// let section = s.plane_section(DVec3::Z * 3.0, DVec3::Z);
	/// let Curve::Circle { radius, .. } = section[0] else { panic!("expected a circle") };
	/// assert!((radius - 4.0).abs() < 1e-12);
	/// ```
	pub fn plane_section(&self, plane_origin: DVec3, plane_normal: DVec3) -> Vec<Curve> {
		let n = plane_normal.normalize_or_zero();
		if n.length_squared() < 0.5 {
			return Vec::new(); // degenerate plane normal
		}
		match *self {
			Surface::Plane { origin, normal } => plane_plane_section(origin, normal.normalize_or_zero(), plane_origin, n),
			Surface::Sphere { center, radius } => sphere_plane_section(center, radius, plane_origin, n),
			Surface::Cylinder { origin, axis, radius } => cylinder_plane_section(origin, axis.normalize_or_zero(), radius, plane_origin, n),
			Surface::Cone { apex, axis, half_angle } => cone_plane_section(apex, axis.normalize_or_zero(), half_angle, plane_origin, n),
			Surface::Torus { center, axis, major, minor } => {
				torus_perp_plane_section(center, axis.normalize_or_zero(), major, minor, plane_origin, n)
			}
		}
	}
}

/// Torus ∩ plane where the plane is PERPENDICULAR to the torus axis: the section is the
/// pair of concentric circles `R ± √(r²−h²)` (`h` = axial offset of the plane), centred on
/// the axis at the cut height. Collapses to one circle of radius `R` where the plane is
/// tangent to the tube (`|h| = r`), and is empty beyond it (`|h| > r`). An OBLIQUE plane
/// gives a quartic (incl. Villarceau) section that is not yet closed-form → empty.
fn torus_perp_plane_section(center: DVec3, axis: DVec3, major: f64, minor: f64, po: DVec3, n: DVec3) -> Vec<Curve> {
	if axis.length_squared() < 0.5 || !major.is_finite() || !minor.is_finite() || major <= 0.0 || minor <= 0.0 {
		return Vec::new();
	}
	if axis.dot(n).abs() < 1.0 - 1e-9 {
		return Vec::new(); // oblique — quartic, not handled
	}
	let h = (po - center).dot(axis); // signed axial offset of the plane from the torus centre
	if h.abs() > minor + 1e-12 {
		return Vec::new(); // plane misses the tube
	}
	let s = (minor * minor - h * h).max(0.0).sqrt();
	let cc = center + axis * h;
	let mut out = vec![Curve::Circle { center: cc, normal: axis, radius: major + s }];
	let inner = major - s;
	if s > 1e-9 && inner > 1e-9 {
		out.push(Curve::Circle { center: cc, normal: axis, radius: inner });
	}
	out
}

/// Intersection line of two planes (empty when parallel).
fn plane_plane_section(o1: DVec3, n1: DVec3, o2: DVec3, n2: DVec3) -> Vec<Curve> {
	if n1.length_squared() < 0.5 {
		return Vec::new();
	}
	let dir = n1.cross(n2);
	let l2 = dir.length_squared();
	if l2 < 1e-18 {
		return Vec::new(); // parallel (coincident or disjoint) — no isolated line
	}
	// A point on both planes: solve the 2×2 system in the plane spanned by n1, n2.
	let d1 = n1.dot(o1);
	let d2 = n2.dot(o2);
	let point = (n2.cross(dir) * d1 + dir.cross(n1) * d2) / l2;
	vec![Curve::Line { origin: point, dir: dir / l2.sqrt() }]
}

/// Sphere ∩ plane: a circle (a single tangent point degenerates to radius 0).
fn sphere_plane_section(center: DVec3, radius: f64, o: DVec3, n: DVec3) -> Vec<Curve> {
	let d = (center - o).dot(n); // signed distance, plane normal is unit
	if d.abs() > radius + 1e-12 {
		return Vec::new();
	}
	let r = (radius * radius - d * d).max(0.0).sqrt();
	vec![Curve::Circle { center: center - n * d, normal: n, radius: r }]
}

/// Cylinder ∩ plane: a circle (⟂ axis), an ellipse (oblique), or one/two lines
/// (∥ axis). Empty when a parallel plane misses the surface.
fn cylinder_plane_section(o: DVec3, axis: DVec3, radius: f64, po: DVec3, n: DVec3) -> Vec<Curve> {
	if axis.length_squared() < 0.5 || !radius.is_finite() || radius <= 0.0 {
		return Vec::new();
	}
	let cos = axis.dot(n); // |cos| = 1 ⟂ plane, 0 ∥ plane
	if cos.abs() < 1e-9 {
		// Plane parallel to the axis: the section is line(s) parallel to the axis.
		// Every axis point sits at the same signed distance from the plane.
		let dd = (o - po).dot(n);
		if dd.abs() > radius + 1e-12 {
			return Vec::new();
		}
		let foot = o - n * dd; // projection of the axis onto the plane
		let perp = axis.cross(n).normalize_or_zero(); // in-plane, ⟂ axis
		let h = (radius * radius - dd * dd).max(0.0).sqrt();
		if h < 1e-9 || perp.length_squared() < 0.5 {
			return vec![Curve::Line { origin: foot, dir: axis }]; // tangent
		}
		return vec![Curve::Line { origin: foot + perp * h, dir: axis }, Curve::Line { origin: foot - perp * h, dir: axis }];
	}
	// Center: where the axis pierces the plane.
	let t = (po - o).dot(n) / cos;
	let center = o + axis * t;
	if cos.abs() > 1.0 - 1e-12 {
		return vec![Curve::Circle { center, normal: n, radius }]; // ⟂ axis
	}
	// Oblique: an ellipse with semi-minor = radius and semi-major = radius/|cos|.
	// The major direction is the cylinder axis projected into the cutting plane.
	let u = (axis - n * cos).normalize_or_zero();
	vec![Curve::Ellipse { center, normal: n, u, a: radius / cos.abs(), b: radius }]
}

/// Cone ∩ plane: the full conic family. A circle (⟂ axis), or — classified by the
/// plane's tilt `|cos∠(n, axis)|` against `sin(half_angle)` — an ellipse (steeper
/// than a generator), a parabola (parallel to one), or a hyperbola (shallower).
/// Through-apex degeneracies (point / line / line-pair) fall out as empty.
fn cone_plane_section(apex: DVec3, axis: DVec3, half_angle: f64, po: DVec3, n: DVec3) -> Vec<Curve> {
	if axis.length_squared() < 0.5 || !half_angle.is_finite() || half_angle <= 0.0 || half_angle >= std::f64::consts::FRAC_PI_2 {
		return Vec::new();
	}
	let acos = axis.dot(n).abs();
	if acos > 1.0 - 1e-9 {
		// Plane perpendicular to the axis: a circle whose radius grows linearly
		// with the axial distance from the apex (radius = s·tan(half_angle)).
		let s = (po - apex).dot(axis);
		if s <= 1e-12 {
			return Vec::new(); // at or behind the apex
		}
		return vec![Curve::Circle { center: apex + axis * s, normal: axis, radius: s * half_angle.tan() }];
	}
	// Through-apex: the conic collapses to the generator line(s) lying in the plane
	// (a point → none, a parabola → one, a hyperbola → a pair). Each is a generator
	// ray from the apex; only its t ≥ 0 half lies on this single-nappe cone.
	if (apex - po).dot(n).abs() < 1e-9 {
		return cone_apex_lines(apex, axis, half_angle, n);
	}
	let sinb = half_angle.sin();
	if (acos - sinb).abs() <= 1e-9 {
		cone_parabola(apex, axis, half_angle, po, n)
	} else if acos > sinb {
		cone_ellipse(apex, axis, half_angle, po, n)
	} else {
		cone_hyperbola(apex, axis, half_angle, po, n)
	}
}

/// Generator lines of a cone through a plane that passes through its apex. Solves
/// `∠(g, axis) = half_angle` for unit in-plane directions `g`: writing
/// `g = cos θ·e1 + sin θ·e2` gives `(e1·axis)cos θ + (e2·axis)sin θ = cos β`, i.e.
/// `R·cos(θ − φ) = cos β` — zero, one, or two solutions.
fn cone_apex_lines(apex: DVec3, axis: DVec3, half_angle: f64, n: DVec3) -> Vec<Curve> {
	let (e1, e2) = perp_basis(n);
	let (aa, bb) = (e1.dot(axis), e2.dot(axis));
	let r = (aa * aa + bb * bb).sqrt(); // |axis projected into the plane|
	if r < 1e-12 {
		return Vec::new(); // axis ⟂ plane: handled by the perpendicular circle path
	}
	let ratio = half_angle.cos() / r;
	if ratio > 1.0 + 1e-12 {
		return Vec::new(); // ellipse-through-apex: just the apex point
	}
	let phi = bb.atan2(aa);
	let thetas: &[f64] = &if ratio > 1.0 - 1e-12 {
		vec![phi] // parabola-through-apex: one tangent generator
	} else {
		let dt = ratio.clamp(-1.0, 1.0).acos();
		vec![phi - dt, phi + dt] // hyperbola-through-apex: two generators
	};
	thetas
		.iter()
		.filter_map(|&theta| {
			let g = (e1 * theta.cos() + e2 * theta.sin()).normalize_or_zero();
			// By construction g·axis = cos β > 0 (ray into the real nappe).
			(g.length_squared() > 0.5 && g.dot(axis) > 0.0).then_some(Curve::Line { origin: apex, dir: g })
		})
		.collect()
}

/// The cone's quadric `(d·axis)² = cos²β·|d|²` (`d = P − apex`) restricted to the
/// cutting plane, returned as the 2-D conic `[A, B, C, D, E, F]` in the orthonormal
/// in-plane basis `(e1, e2)` (so `P = po + x·e1 + y·e2`).
fn cone_plane_conic(apex: DVec3, axis: DVec3, half_angle: f64, po: DVec3, n: DVec3) -> (DVec3, DVec3, [f64; 6]) {
	let (e1, e2) = perp_basis(n);
	let w = po - apex;
	let k = half_angle.cos().powi(2);
	let (p0, p1, p2) = (axis.dot(w), axis.dot(e1), axis.dot(e2));
	let (q0, q1, q2) = (w.length_squared(), w.dot(e1), w.dot(e2));
	let coeffs = [
		p1 * p1 - k,              // A x²
		2.0 * p1 * p2,            // B xy
		p2 * p2 - k,              // C y²
		2.0 * (p0 * p1 - k * q1), // D x
		2.0 * (p0 * p2 - k * q2), // E y
		p0 * p0 - k * q0,         // F
	];
	(e1, e2, coeffs)
}

/// Eigenvalues `(λ_max, λ_min)` of the symmetric form `[[A, B/2], [B/2, C]]`.
fn conic_eigvals(a: f64, b: f64, c: f64) -> (f64, f64) {
	let half_tr = (a + c) * 0.5;
	let disc = (half_tr * half_tr - (a * c - b * b * 0.25)).max(0.0).sqrt();
	(half_tr + disc, half_tr - disc)
}

/// Centre of a central conic (solves `[[2A, B], [B, 2C]]·[xc, yc] = [-D, -E]`).
fn conic_center(a: f64, b: f64, c: f64, d: f64, e: f64, det: f64) -> (f64, f64) {
	((-2.0 * c * d + b * e) / det, (-2.0 * a * e + b * d) / det)
}

/// Oblique cone ∩ plane → ellipse (centre + eigen-axes of the in-plane conic).
fn cone_ellipse(apex: DVec3, axis: DVec3, half_angle: f64, po: DVec3, n: DVec3) -> Vec<Curve> {
	let (e1, e2, [a, b, c, d, e, f]) = cone_plane_conic(apex, axis, half_angle, po, n);
	let det = 4.0 * a * c - b * b;
	if det.abs() < 1e-15 {
		return Vec::new();
	}
	let (xc, yc) = conic_center(a, b, c, d, e, det);
	let fc = a * xc * xc + b * xc * yc + c * yc * yc + d * xc + e * yc + f;
	// Semi-axis along eigenvector i is √(−fc/λᵢ).
	let (l1, l2) = conic_eigvals(a, b, c);
	let (r1sq, r2sq) = (-fc / l1, -fc / l2);
	if !(r1sq > 0.0 && r2sq > 0.0 && r1sq.is_finite() && r2sq.is_finite()) {
		return Vec::new(); // not a real bounded ellipse
	}
	let (r1, r2) = (r1sq.sqrt(), r2sq.sqrt());
	let ev1 = conic_eigvec(a, b, c, l1);
	let (semi_major, semi_minor, major2d) = if r1 >= r2 {
		(r1, r2, ev1)
	} else {
		(r2, r1, DVec2::new(-ev1.y, ev1.x)) // λ2 eigenvector ⟂ λ1's
	};
	let u = (e1 * major2d.x + e2 * major2d.y).normalize_or_zero();
	let center = po + e1 * xc + e2 * yc;
	if u.length_squared() < 0.5 || axis.dot(center - apex) <= 0.0 {
		return Vec::new(); // degenerate, or on the phantom nappe (behind the apex)
	}
	vec![Curve::Ellipse { center, normal: n, u, a: semi_major, b: semi_minor }]
}

/// Oblique cone ∩ plane → one hyperbola branch (the one on the real nappe).
fn cone_hyperbola(apex: DVec3, axis: DVec3, half_angle: f64, po: DVec3, n: DVec3) -> Vec<Curve> {
	let (e1, e2, [a, b, c, d, e, f]) = cone_plane_conic(apex, axis, half_angle, po, n);
	let det = 4.0 * a * c - b * b;
	if det.abs() < 1e-15 {
		return Vec::new();
	}
	let (xc, yc) = conic_center(a, b, c, d, e, det);
	let fc = a * xc * xc + b * xc * yc + c * yc * yc + d * xc + e * yc + f;
	if fc.abs() < 1e-18 {
		return Vec::new(); // through-apex degenerate (a line pair, not a hyperbola)
	}
	// Eigenvalues have opposite sign; the transverse axis is the one whose √(−fc/λ)
	// is real (the conjugate axis takes the imaginary √).
	let (l1, l2) = conic_eigvals(a, b, c);
	let (p1sq, p2sq) = (-fc / l1, -fc / l2);
	let (a_sq, b_sq, ev_t) = if p1sq > 0.0 { (p1sq, -p2sq, conic_eigvec(a, b, c, l1)) } else { (p2sq, -p1sq, conic_eigvec(a, b, c, l2)) };
	if !(a_sq > 0.0 && b_sq > 0.0 && a_sq.is_finite() && b_sq.is_finite()) {
		return Vec::new();
	}
	let (a_h, b_h) = (a_sq.sqrt(), b_sq.sqrt());
	let center = po + e1 * xc + e2 * yc;
	let mut u = (e1 * ev_t.x + e2 * ev_t.y).normalize_or_zero();
	if u.length_squared() < 0.5 {
		return Vec::new();
	}
	// Of the two vertices ±u·a, keep the branch whose vertex sits on the real nappe.
	if axis.dot((center - u * a_h) - apex) > axis.dot((center + u * a_h) - apex) {
		u = -u;
	}
	if axis.dot((center + u * a_h) - apex) <= 0.0 {
		return Vec::new(); // the cut misses the real nappe
	}
	vec![Curve::Hyperbola { center, normal: n, u, a: a_h, b: b_h }]
}

/// Oblique cone ∩ plane → parabola (plane parallel to a generator). Rotates the
/// in-plane conic to principal axes: the zero-eigenvalue direction is the opening
/// axis, the other carries the quadratic term.
fn cone_parabola(apex: DVec3, axis: DVec3, half_angle: f64, po: DVec3, n: DVec3) -> Vec<Curve> {
	let (e1, e2, [a, b, c, d, e, f]) = cone_plane_conic(apex, axis, half_angle, po, n);
	let (l1, l2) = conic_eigvals(a, b, c);
	let (lam, _) = if l1.abs() >= l2.abs() { (l1, l2) } else { (l2, l1) };
	if lam.abs() < 1e-15 {
		return Vec::new();
	}
	let ev_q = conic_eigvec(a, b, c, lam); // quadratic direction ξ
	let ev_l = DVec2::new(-ev_q.y, ev_q.x); // opening direction η (⟂ ξ)
	let d_xi = d * ev_q.x + e * ev_q.y;
	let d_eta = d * ev_l.x + e * ev_l.y;
	if d_eta.abs() < 1e-15 {
		return Vec::new(); // no linear term to open along → degenerate
	}
	// η = −(λ/d_η)·ξ'² + η₀ after completing the square (ξ' = ξ + d_ξ/2λ).
	let xi_v = -d_xi / (2.0 * lam);
	let eta_v = -(f - d_xi * d_xi / (4.0 * lam)) / d_eta;
	let xv = ev_q.x * xi_v + ev_l.x * eta_v;
	let yv = ev_q.y * xi_v + ev_l.y * eta_v;
	let vertex = po + e1 * xv + e2 * yv;
	let open_sign = if (lam / d_eta) > 0.0 { -1.0 } else { 1.0 };
	let axis_dir2 = ev_l * open_sign;
	let axis_dir = (e1 * axis_dir2.x + e2 * axis_dir2.y).normalize_or_zero();
	let width_dir = (e1 * ev_q.x + e2 * ev_q.y).normalize_or_zero();
	let focal = (d_eta / (4.0 * lam)).abs();
	if !(focal > 0.0 && focal.is_finite())
		|| axis_dir.length_squared() < 0.5
		|| width_dir.length_squared() < 0.5
		|| axis.dot(vertex - apex) <= 1e-12
	{
		return Vec::new();
	}
	vec![Curve::Parabola { vertex, axis: axis_dir, dir: width_dir, focal }]
}

/// Unit eigenvector of the symmetric matrix `[[a, b/2], [b/2, c]]` for eigenvalue `l`.
fn conic_eigvec(a: f64, b: f64, c: f64, l: f64) -> DVec2 {
	let bx = b * 0.5;
	let v = if bx.abs() > 1e-15 {
		DVec2::new(bx, l - a)
	} else if (l - a).abs() <= (l - c).abs() {
		DVec2::new(1.0, 0.0) // already diagonal
	} else {
		DVec2::new(0.0, 1.0)
	};
	v.normalize_or_zero()
}

/// A face-local 2-D **parameter-space chart** of a curved analytic surface — the
/// domain in which a *warped* (non-planar) curved-tagged face boundary can be
/// ear-clipped without folding.
///
/// A boolean's seam snapper moves cut-seam vertices onto the exact
/// surface–surface intersection; the incident curved facets then leave their
/// chord planes by up to the facet sagitta, and ear-clipping such a polygon in a
/// projection plane can self-intersect (the measured W3 fuzz-failure class —
/// see `ROBUSTNESS.md`). In the surface's own parameters the same boundary is a
/// simple polygon whenever it bounds a simple region *on the surface*, so the
/// clip cannot fold. Charts are near-isometric (scaled by the local radii) so
/// the 2-D ear/containment tolerances keep their model-unit meaning:
///
/// - **Cylinder** → unrolled `(r·θ̃, z)`; the angular seam is handled by
///   anchoring `θ̃ = 0` on the mean radial direction of the ring (injective for
///   faces spanning < 2π — every facet-born face here spans ≪ π).
/// - **Sphere** → **gnomonic** projection about the ring's mean direction:
///   central projection from the centre onto the tangent plane. This removes the
///   (θ, φ) pole singularity instead of special-casing it — a polar cap's mean
///   direction IS the pole, where the gnomonic chart is best-conditioned;
///   injective on the open hemisphere.
/// - **Cone** → isometric development (unroll) `ρ·(cos, sin)(sin α · θ̃)` about
///   the mean radial anchor; exact at the apex (`ρ = 0`).
/// - **Torus** → `(R·θ̃, r·ψ̃)`, both angles unwrapped about the ring's mean
///   ring/tube directions (injective for patches spanning < 2π in each angle).
///
/// The chart is only consulted for *which diagonals to clip* — emitted triangles
/// always keep the input ring's 3-D vertex order/winding — so chart orientation
/// conventions never leak into face orientation.
#[derive(Clone, Copy, Debug)]
pub(crate) enum SurfaceChart {
	Cylinder { origin: DVec3, axis: DVec3, radius: f64, e1: DVec3, e2: DVec3 },
	Sphere { center: DVec3, radius: f64, w: DVec3, e1: DVec3, e2: DVec3 },
	Cone { apex: DVec3, sin_half: f64, e1: DVec3, e2: DVec3 },
	Torus { center: DVec3, axis: DVec3, major: f64, minor: f64, e1: DVec3, e2: DVec3, psi_anchor: DVec2 },
}

/// Boundary deviation from the face plane above which a curved-tagged face ring
/// counts as WARPED and is clipped in its [`SurfaceChart`] instead of a
/// projection plane. Sits above stitch noise (a boolean-result face is planar
/// only to the weld/heal scale, ≤ ~4e-7) and far below the sagitta-scale warp
/// (~1e-2..1e-3) that seam snapping introduces, so every exactly planar face —
/// all faces the kernel produced before the W5 snap relaxation — keeps the
/// projection-plane path byte-identically. (A prior prototype that charted every
/// curved face unconditionally re-diagonalised planar chord facets too, which
/// shifted marginal fuzz chains; engaging on measured warp only keeps the
/// no-snap corpus bit-stable.)
pub(crate) const CURVED_WARP_EPS: f64 = 1e-6;

impl SurfaceChart {
	/// The chart to clip a measurably **warped** curved-face ring in — `None` for
	/// a plane-tagged, still-planar (within [`CURVED_WARP_EPS`] of the plane
	/// through `ring[0]` ⟂ `normal`), or chart-degenerate ring, in which case the
	/// caller keeps its projection-plane path, byte-identical to the
	/// pre-parameter-space behaviour.
	pub(crate) fn for_warped_ring(surface: &Surface, ring: &[DVec3], normal: DVec3) -> Option<SurfaceChart> {
		if ring.len() < 3 || matches!(surface, Surface::Plane { .. }) {
			return None;
		}
		let planarity = ring.iter().map(|&p| (p - ring[0]).dot(normal).abs()).fold(0.0, f64::max);
		if planarity <= CURVED_WARP_EPS {
			return None;
		}
		SurfaceChart::new(surface, ring)
	}

	/// Build the chart for `surface`, anchored on the boundary `ring` (its mean
	/// radial / normal direction becomes the chart origin, so per-vertex angles
	/// unwrap into `(−π, π]` with no seam crossing the face). `None` for a plane
	/// (the projection-plane path is already exact there) or a degenerate ring
	/// (e.g. radial directions cancelling out — a ≥ π span this chart must not
	/// guess about).
	pub(crate) fn new(surface: &Surface, ring: &[DVec3]) -> Option<SurfaceChart> {
		// Mean of `dirs`, normalised — the deterministic unwrap anchor.
		let mean_dir = |dirs: &mut dyn Iterator<Item = DVec3>| -> Option<DVec3> {
			let sum: DVec3 = dirs.fold(DVec3::ZERO, |a, d| a + d.normalize_or_zero());
			(sum.length_squared() > 1e-12).then(|| sum.normalize())
		};
		match *surface {
			Surface::Plane { .. } => None,
			Surface::Cylinder { origin, axis, radius } => {
				let axis = axis.normalize_or_zero();
				if axis.length_squared() < 0.5 || !radius.is_finite() || radius <= 0.0 {
					return None;
				}
				let e1 = mean_dir(&mut ring.iter().map(|&p| {
					let rel = p - origin;
					rel - axis * rel.dot(axis)
				}))?;
				Some(SurfaceChart::Cylinder { origin, axis, radius, e1, e2: axis.cross(e1) })
			}
			Surface::Sphere { center, radius } => {
				if !radius.is_finite() || radius <= 0.0 {
					return None;
				}
				let w = mean_dir(&mut ring.iter().map(|&p| p - center))?;
				let (e1, e2) = perp_basis(w);
				Some(SurfaceChart::Sphere { center, radius, w, e1, e2 })
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
				// The development plane's polar axis is `e1` by construction; only the
				// angular scale (sin α) and the slant distance are needed afterwards.
				Some(SurfaceChart::Cone { apex, sin_half: half_angle.sin(), e1, e2: axis.cross(e1) })
			}
			Surface::Torus { center, axis, major, minor } => {
				let axis = axis.normalize_or_zero();
				if axis.length_squared() < 0.5 || !major.is_finite() || major <= 0.0 || !minor.is_finite() || minor <= 0.0 {
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
				Some(SurfaceChart::Torus { center, axis, major, minor, e1, e2, psi_anchor: psi_sum.normalize() })
			}
		}
	}

	/// Chart coordinates of one (near-surface) point. `None` when the point falls
	/// outside the chart's injective domain (a gnomonic point at/behind the
	/// horizon, a point on a cylinder/cone/torus axis) — the caller then falls
	/// back to the projection-plane path rather than clip in a broken chart.
	pub(crate) fn uv(&self, p: DVec3) -> Option<DVec2> {
		let out = match *self {
			SurfaceChart::Cylinder { origin, axis, radius, e1, e2 } => {
				let rel = p - origin;
				let (x, y) = (rel.dot(e1), rel.dot(e2));
				if x * x + y * y < 1e-24 {
					return None; // on the axis: θ undefined
				}
				DVec2::new(radius * y.atan2(x), rel.dot(axis))
			}
			SurfaceChart::Sphere { center, radius, w, e1, e2 } => {
				let rel = p - center;
				let d = rel.dot(w);
				if d <= 1e-9 * radius {
					return None; // at/behind the gnomonic horizon
				}
				DVec2::new(radius * rel.dot(e1) / d, radius * rel.dot(e2) / d)
			}
			SurfaceChart::Cone { apex, sin_half, e1, e2 } => {
				let rel = p - apex;
				let rho = rel.length();
				if rho < 1e-15 {
					return Some(DVec2::ZERO); // the apex develops to the origin exactly
				}
				let (x, y) = (rel.dot(e1), rel.dot(e2));
				if x * x + y * y < 1e-24 {
					return None; // on the axis (θ undefined) yet off the apex
				}
				let dev = sin_half * y.atan2(x);
				DVec2::new(rho * dev.cos(), rho * dev.sin())
			}
			SurfaceChart::Torus { center, axis, major, minor, e1, e2, psi_anchor } => {
				let rel = p - center;
				let (x, y) = (rel.dot(e1), rel.dot(e2));
				if x * x + y * y < 1e-24 {
					return None; // on the torus axis: θ undefined
				}
				let h = rel.dot(axis);
				let ring_dir = (rel - axis * h).normalize_or_zero();
				let d = p - (center + ring_dir * major);
				let tube = DVec2::new(d.dot(ring_dir), h);
				if tube.length_squared() < 1e-24 {
					return None; // on the tube's spine circle: ψ undefined
				}
				// ψ̃ = ψ − ψ₀ via the 2-D rotation (cos, sin)·(anchor conjugate).
				let psi = DVec2::new(tube.x * psi_anchor.x + tube.y * psi_anchor.y, tube.y * psi_anchor.x - tube.x * psi_anchor.y);
				DVec2::new(major * y.atan2(x), minor * psi.y.atan2(psi.x))
			}
		};
		out.is_finite().then_some(out)
	}

	/// Chart coordinates of a whole ring, or `None` if any vertex falls outside
	/// the chart's injective domain.
	pub(crate) fn uv_ring(&self, ring: &[DVec3]) -> Option<Vec<DVec2>> {
		ring.iter().map(|&p| self.uv(p)).collect()
	}
}

/// A closed-form analytic curve.
#[derive(Clone, Copy, Debug)]
pub enum Curve {
	Line {
		origin: DVec3,
		dir: DVec3,
	},
	Circle {
		center: DVec3,
		normal: DVec3,
		radius: f64,
	},
	/// An ellipse `center + u·a·cos(t) + (normal×u)·b·sin(t)`, where `u` is the unit
	/// semi-major direction, `a >= b > 0` the semi-axes, and `normal` the plane normal.
	Ellipse {
		center: DVec3,
		normal: DVec3,
		u: DVec3,
		a: f64,
		b: f64,
	},
	/// A parabola `vertex + dir·t + axis·t²/(4·focal)` opening along unit `axis`,
	/// with unit `dir ⟂ axis` spanning the width and focal length `focal > 0`.
	Parabola {
		vertex: DVec3,
		axis: DVec3,
		dir: DVec3,
		focal: f64,
	},
	/// One branch of a hyperbola `center + u·a·cosh(s) + (normal×u)·b·sinh(s)`, with
	/// unit transverse direction `u` (pointing at the vertex) and semi-axes `a, b > 0`.
	Hyperbola {
		center: DVec3,
		normal: DVec3,
		u: DVec3,
		a: f64,
		b: f64,
	},
}

impl Curve {
	/// Position at parameter `t` (arc length for `Line`, radians for `Circle` /
	/// `Ellipse`).
	pub fn point_at(&self, t: f64) -> DVec3 {
		match *self {
			Curve::Line { origin, dir } => origin + dir * t,
			Curve::Circle { center, normal, radius } => {
				let (e1, e2) = perp_basis(normal);
				center + (e1 * t.cos() + e2 * t.sin()) * radius
			}
			Curve::Ellipse { center, normal, u, a, b } => {
				let v = normal.cross(u).normalize_or_zero();
				center + u * (a * t.cos()) + v * (b * t.sin())
			}
			Curve::Parabola { vertex, axis, dir, focal } => vertex + dir * t + axis * (t * t / (4.0 * focal)),
			Curve::Hyperbola { center, normal, u, a, b } => {
				let v = normal.cross(u).normalize_or_zero();
				center + u * (a * t.cosh()) + v * (b * t.sinh())
			}
		}
	}

	/// Unit tangent at parameter `t`.
	pub fn tangent_at(&self, t: f64) -> DVec3 {
		match *self {
			Curve::Line { dir, .. } => dir.normalize_or_zero(),
			Curve::Circle { normal, .. } => {
				let (e1, e2) = perp_basis(normal);
				(-e1 * t.sin() + e2 * t.cos()).normalize_or_zero()
			}
			Curve::Ellipse { normal, u, a, b, .. } => {
				let v = normal.cross(u).normalize_or_zero();
				(u * (-a * t.sin()) + v * (b * t.cos())).normalize_or_zero()
			}
			Curve::Parabola { axis, dir, focal, .. } => (dir + axis * (t / (2.0 * focal))).normalize_or_zero(),
			Curve::Hyperbola { normal, u, a, b, .. } => {
				let v = normal.cross(u).normalize_or_zero();
				(u * (a * t.sinh()) + v * (b * t.cosh())).normalize_or_zero()
			}
		}
	}

	/// Speed `|C'(t)|` — the magnitude of the un-normalised derivative.
	fn speed(&self, t: f64) -> f64 {
		match *self {
			Curve::Line { dir, .. } => dir.length(),
			Curve::Circle { radius, .. } => radius.abs(),
			Curve::Ellipse { normal, u, a, b, .. } => {
				let v = normal.cross(u).normalize_or_zero();
				(u * (-a * t.sin()) + v * (b * t.cos())).length()
			}
			Curve::Parabola { axis, dir, focal, .. } => (dir + axis * (t / (2.0 * focal))).length(),
			Curve::Hyperbola { normal, u, a, b, .. } => {
				let v = normal.cross(u).normalize_or_zero();
				(u * (a * t.sinh()) + v * (b * t.cosh())).length()
			}
		}
	}

	/// Exact arc length between parameters `t0` and `t1`. Closed-form for `Line` and
	/// `Circle` (constant speed); for the other conics it integrates the speed `|C'(t)|`
	/// with composite Gauss–Legendre quadrature, which is machine-exact for these smooth
	/// analytic curves — the exact measure of a curved edge an AI needs for dimensioning.
	pub fn length(&self, t0: f64, t1: f64) -> f64 {
		match *self {
			Curve::Line { dir, .. } => dir.length() * (t1 - t0).abs(),
			Curve::Circle { radius, .. } => radius.abs() * (t1 - t0).abs(),
			_ => gauss_legendre_5(|t| self.speed(t), t0, t1),
		}
	}

	/// Transform the curve by `m` (rigid + uniform scale).
	pub fn transformed(&self, m: DAffine3) -> Curve {
		let scale = m.matrix3.x_axis.length();
		match *self {
			Curve::Line { origin, dir } => Curve::Line { origin: m.transform_point3(origin), dir: m.transform_vector3(dir) },
			Curve::Circle { center, normal, radius } => Curve::Circle {
				center: m.transform_point3(center),
				normal: m.transform_vector3(normal).normalize_or_zero(),
				radius: radius * scale,
			},
			Curve::Ellipse { center, normal, u, a, b } => Curve::Ellipse {
				center: m.transform_point3(center),
				normal: m.transform_vector3(normal).normalize_or_zero(),
				u: m.transform_vector3(u).normalize_or_zero(),
				a: a * scale,
				b: b * scale,
			},
			Curve::Parabola { vertex, axis, dir, focal } => Curve::Parabola {
				vertex: m.transform_point3(vertex),
				axis: m.transform_vector3(axis).normalize_or_zero(),
				dir: m.transform_vector3(dir).normalize_or_zero(),
				focal: focal * scale,
			},
			Curve::Hyperbola { center, normal, u, a, b } => Curve::Hyperbola {
				center: m.transform_point3(center),
				normal: m.transform_vector3(normal).normalize_or_zero(),
				u: m.transform_vector3(u).normalize_or_zero(),
				a: a * scale,
				b: b * scale,
			},
		}
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	/// Whether any two non-adjacent edges of the closed 2-D `ring` properly cross —
	/// the fold the parameter-space chart exists to prevent.
	fn ring_self_intersects(ring: &[DVec2]) -> bool {
		let n = ring.len();
		let cross = |a: DVec2, b: DVec2, c: DVec2, d: DVec2| {
			let o = |p: DVec2, q: DVec2, r: DVec2| (q - p).perp_dot(r - p);
			o(a, b, c) * o(a, b, d) < 0.0 && o(c, d, a) * o(c, d, b) < 0.0
		};
		for i in 0..n {
			for j in i + 1..n {
				if j == i || (j + 1) % n == i || (i + 1) % n == j {
					continue;
				}
				if cross(ring[i], ring[(i + 1) % n], ring[j], ring[(j + 1) % n]) {
					return true;
				}
			}
		}
		false
	}

	/// A warped facet-like boundary ring sampled ON `surface`: a (u, v)-rectangle in
	/// the surface's natural parameters via [`Surface::point_at`], densified along
	/// the u (angular) sides so the ring leaves the chord plane by the sagitta.
	fn warped_patch_ring(surface: &Surface, u0: f64, u1: f64, v0: f64, v1: f64, n: usize) -> Vec<DVec3> {
		let mut ring = Vec::new();
		for k in 0..=n {
			ring.push(surface.point_at(u0 + (u1 - u0) * k as f64 / n as f64, v0));
		}
		for k in 1..=n {
			ring.push(surface.point_at(u1, v0 + (v1 - v0) * k as f64 / n as f64));
		}
		for k in 1..=n {
			ring.push(surface.point_at(u1 - (u1 - u0) * k as f64 / n as f64, v1));
		}
		for k in 1..n {
			ring.push(surface.point_at(u0, v1 - (v1 - v0) * k as f64 / n as f64));
		}
		ring
	}

	/// Newell normal (unit) of a 3-D ring — the same reference the triangulators use.
	fn newell(poly: &[DVec3]) -> DVec3 {
		let mut nrm = DVec3::ZERO;
		let len = poly.len();
		for i in 0..len {
			let (c, d) = (poly[i], poly[(i + 1) % len]);
			nrm.x += (c.y - d.y) * (c.z + d.z);
			nrm.y += (c.z - d.z) * (c.x + d.x);
			nrm.z += (c.x - d.x) * (c.y + d.y);
		}
		nrm.normalize_or_zero()
	}

	#[test]
	fn surface_charts_keep_warped_on_surface_rings_simple() {
		// One warped patch ring per curved surface kind — each ring's vertices lie ON
		// the true surface (so it bounds a simple surface region) yet leave the chord
		// plane by far more than CURVED_WARP_EPS. The chart must engage, map every
		// vertex, and produce a SIMPLE polygon — projection-plane ear-clipping is
		// exactly what folds on these. The sphere case wraps a ring AROUND the pole
		// (the (θ, φ) singularity) — the gnomonic chart's reason for existence; the
		// cylinder/torus rings straddle the atan2 ±π seam (anchored mean direction
		// −X) — the seam-unwrap care.
		let cyl = Surface::Cylinder { origin: DVec3::ZERO, axis: DVec3::Z, radius: 2.0 };
		let sph = Surface::Sphere { center: DVec3::ZERO, radius: 3.0 };
		let cone = Surface::Cone { apex: DVec3::ZERO, axis: DVec3::Z, half_angle: 0.4 };
		let tor = Surface::Torus { center: DVec3::ZERO, axis: DVec3::Z, major: 5.0, minor: 1.5 };
		let pi = std::f64::consts::PI;
		let cases: Vec<(&str, &Surface, Vec<DVec3>)> = vec![
			// 60° cylinder band straddling θ = π (the atan2 seam).
			("cylinder", &cyl, warped_patch_ring(&cyl, pi - 0.5, pi + 0.5, 0.0, 1.0, 6)),
			// Polar cap boundary: v (polar angle) from 0.15 to 0.7 over a 90° azimuth
			// wedge would dodge the pole — instead take a full small circle AROUND it.
			("sphere polar cap", &sph, (0..24).map(|k| sph.point_at(2.0 * pi * k as f64 / 24.0, 0.35 + 0.1 * ((k % 2) as f64))).collect()),
			("cone", &cone, warped_patch_ring(&cone, -0.4, 0.4, 1.0, 2.0, 6)),
			("torus", &tor, warped_patch_ring(&tor, pi - 0.4, pi + 0.4, 0.3, 1.2, 6)),
		];
		let mut report = String::new();
		for (name, surface, ring) in &cases {
			let nrm = newell(ring);
			let planarity = ring.iter().map(|&p| (p - ring[0]).dot(nrm).abs()).fold(0.0, f64::max);
			let chart = SurfaceChart::for_warped_ring(surface, ring, nrm);
			let uv = chart.as_ref().and_then(|c| c.uv_ring(ring));
			let simple = uv.as_ref().map(|r| !ring_self_intersects(r));
			report.push_str(&format!(
				"{name}: planarity={planarity:.2e} chart={} mapped={} simple={:?}\n",
				chart.is_some(),
				uv.is_some(),
				simple
			));
			if planarity <= CURVED_WARP_EPS || chart.is_none() || uv.is_none() || simple != Some(true) {
				panic!("a warped on-surface ring must chart to a simple parameter-space polygon:\n{report}");
			}
		}
	}

	#[test]
	fn surface_chart_refuses_planar_rings_and_off_domain_points() {
		// Byte-identity contract: a PLANAR curved-tagged ring (every chord facet the
		// kernel built before the W5 snap relaxation) must NOT engage the chart —
		// `for_warped_ring` is the gate that keeps the old projection path bit-stable.
		// And a ring the chart cannot map injectively (a gnomonic point at the
		// horizon) must refuse rather than guess.
		let cyl = Surface::Cylinder { origin: DVec3::ZERO, axis: DVec3::Z, radius: 2.0 };
		// A flat chord facet of the cylinder: two generators at θ = ±0.3.
		let chord = vec![cyl.point_at(-0.3, 0.0), cyl.point_at(0.3, 0.0), cyl.point_at(0.3, 1.0), cyl.point_at(-0.3, 1.0)];
		let flat_refused = SurfaceChart::for_warped_ring(&cyl, &chord, newell(&chord)).is_none();
		// Gnomonic horizon: a ring spanning a great circle's diameter has a vertex
		// with rel·w ≤ 0 — uv must refuse it (the caller falls back).
		let sph = Surface::Sphere { center: DVec3::ZERO, radius: 1.0 };
		let hemi = vec![DVec3::X, DVec3::Y, -DVec3::X, DVec3::Z];
		let horizon_refused = SurfaceChart::new(&sph, &hemi).map(|c| c.uv_ring(&hemi)).is_none_or(|uv| uv.is_none());
		// A plane never charts; a degenerate (axis-cancelling) cylinder ring never charts.
		let pl = Surface::Plane { origin: DVec3::ZERO, normal: DVec3::Z };
		let plane_refused = SurfaceChart::new(&pl, &chord).is_none();
		assert!(
			flat_refused && horizon_refused && plane_refused,
			"chart gates: flat chord facet refused={flat_refused}, gnomonic horizon refused={horizon_refused}, plane refused={plane_refused}"
		);
	}

	#[test]
	fn cylinder_chart_is_the_isometric_unroll() {
		// The chart is near-isometric: an on-surface (θ, z) rectangle's chart image
		// must be the exact (r·Δθ) × Δz rectangle — chart-space tolerances keep their
		// model-unit meaning (the ear-clip's sliver/containment epsilons).
		let cyl = Surface::Cylinder { origin: DVec3::new(1.0, -2.0, 0.5), axis: DVec3::Y, radius: 3.0 };
		let ring = warped_patch_ring(&cyl, 0.2, 1.4, -1.0, 2.0, 8);
		let chart = SurfaceChart::new(&cyl, &ring).expect("cylinder ring charts");
		let uv = chart.uv_ring(&ring).expect("all vertices map");
		let (mut lo, mut hi) = (DVec2::splat(f64::INFINITY), DVec2::splat(f64::NEG_INFINITY));
		for p in &uv {
			lo = lo.min(*p);
			hi = hi.max(*p);
		}
		let extent = hi - lo;
		let expect = DVec2::new(3.0 * 1.2, 3.0);
		assert!(
			(extent - expect).length() < 1e-9,
			"unrolled (rθ, z) extent must be ({:.3}, {:.3}), got ({:.3}, {:.3})",
			expect.x,
			expect.y,
			extent.x,
			extent.y
		);
	}
}
