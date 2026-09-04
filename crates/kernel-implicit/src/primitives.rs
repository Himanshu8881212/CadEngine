// Copyright (c) LMCAD. Licensed under the MIT License.

//! Analytic SDF primitives.
//!
//! Each primitive is a small data struct implementing [`Sdf`] with a closed-form
//! exact distance, a tight bound, and (where cheap) an analytic gradient. The
//! distance formulas follow the standard analytic SDFs (after Inigo Quilez);
//! every one is exact Euclidean signed distance except [`Gyroid`] (a TPMS field).

use kernel_core::math::{Aabb, DVec2, DVec3, Vec2, Vec3};
use kernel_core::sdf::Sdf;

/// A solid sphere.
#[derive(Clone, Copy, Debug)]
pub struct Sphere {
	pub center: Vec3,
	pub radius: f32,
}

impl Sphere {
	pub fn new(center: Vec3, radius: f32) -> Self {
		Self { center, radius }
	}
}

impl Sdf for Sphere {
	fn distance(&self, p: Vec3) -> f32 {
		(p - self.center).length() - self.radius
	}
	fn distance64(&self, p: DVec3) -> f64 {
		(p - self.center.as_dvec3()).length() - self.radius as f64
	}
	fn bounds(&self) -> Aabb {
		Aabb::from_center_half_extent(self.center, Vec3::splat(self.radius))
	}
	fn gradient(&self, p: Vec3) -> Vec3 {
		(p - self.center).normalize_or_zero()
	}
}

/// An axis-aligned box (cuboid) given by centre and half extents.
#[derive(Clone, Copy, Debug)]
pub struct Cuboid {
	pub center: Vec3,
	pub half: Vec3,
}

impl Cuboid {
	pub fn new(center: Vec3, half: Vec3) -> Self {
		Self { center, half }
	}
	/// Construct from minimum and maximum corners.
	pub fn from_corners(min: Vec3, max: Vec3) -> Self {
		Self { center: (min + max) * 0.5, half: (max - min).abs() * 0.5 }
	}
}

impl Sdf for Cuboid {
	fn distance(&self, p: Vec3) -> f32 {
		let q = (p - self.center).abs() - self.half;
		q.max(Vec3::ZERO).length() + q.max_element().min(0.0)
	}
	fn distance64(&self, p: DVec3) -> f64 {
		let q = (p - self.center.as_dvec3()).abs() - self.half.as_dvec3();
		q.max(DVec3::ZERO).length() + q.max_element().min(0.0)
	}
	fn bounds(&self) -> Aabb {
		Aabb::from_center_half_extent(self.center, self.half)
	}
	fn gradient(&self, p: Vec3) -> Vec3 {
		let d = p - self.center;
		let s = Vec3::new(d.x.signum(), d.y.signum(), d.z.signum());
		let q = d.abs() - self.half;
		if q.max_element() > 0.0 {
			// Exterior: normal aligns with the positive components of q.
			(q.max(Vec3::ZERO) * s).normalize_or_zero()
		} else {
			// Interior: nearest face is the axis whose q is closest to zero.
			let axis = if q.x >= q.y && q.x >= q.z {
				Vec3::X
			} else if q.y >= q.z {
				Vec3::Y
			} else {
				Vec3::Z
			};
			axis * s
		}
	}
}

/// A capped cylinder between endpoints `a` and `b` with the given radius.
#[derive(Clone, Copy, Debug)]
pub struct Cylinder {
	pub a: Vec3,
	pub b: Vec3,
	pub radius: f32,
}

impl Cylinder {
	pub fn new(a: Vec3, b: Vec3, radius: f32) -> Self {
		Self { a, b, radius }
	}
}

impl Sdf for Cylinder {
	fn distance(&self, p: Vec3) -> f32 {
		let ba = self.b - self.a;
		let pa = p - self.a;
		let baba = ba.dot(ba);
		if baba < 1e-12 {
			// Degenerate zero-length axis: behave as a sphere of the radius.
			return pa.length() - self.radius;
		}
		let paba = pa.dot(ba);
		let x = (pa * baba - ba * paba).length() - self.radius * baba;
		let y = (paba - baba * 0.5).abs() - baba * 0.5;
		let x2 = x * x;
		let y2 = y * y * baba;
		let d = if x.max(y) < 0.0 { -(x2.min(y2)) } else { (if x > 0.0 { x2 } else { 0.0 }) + (if y > 0.0 { y2 } else { 0.0 }) };
		d.signum() * d.abs().sqrt() / baba
	}
	fn distance64(&self, p: DVec3) -> f64 {
		let ba = self.b.as_dvec3() - self.a.as_dvec3();
		let pa = p - self.a.as_dvec3();
		let baba = ba.dot(ba);
		if baba < 1e-12 {
			return pa.length() - self.radius as f64;
		}
		let paba = pa.dot(ba);
		let x = (pa * baba - ba * paba).length() - self.radius as f64 * baba;
		let y = (paba - baba * 0.5).abs() - baba * 0.5;
		let (x2, y2) = (x * x, y * y * baba);
		let d = if x.max(y) < 0.0 { -(x2.min(y2)) } else { (if x > 0.0 { x2 } else { 0.0 }) + (if y > 0.0 { y2 } else { 0.0 }) };
		d.signum() * d.abs().sqrt() / baba
	}
	fn bounds(&self) -> Aabb {
		Aabb::from_points(&[self.a, self.b]).pad(self.radius)
	}
}

/// A capped cone (frustum) from disk (`a`, radius `ra`) to disk (`b`, radius `rb`).
///
/// `rb = 0` gives a sharp cone; `ra = rb` gives a cylinder.
#[derive(Clone, Copy, Debug)]
pub struct Cone {
	pub a: Vec3,
	pub b: Vec3,
	pub ra: f32,
	pub rb: f32,
}

impl Cone {
	pub fn new(a: Vec3, b: Vec3, ra: f32, rb: f32) -> Self {
		Self { a, b, ra, rb }
	}
}

impl Sdf for Cone {
	fn distance(&self, p: Vec3) -> f32 {
		let rba = self.rb - self.ra;
		let ba = self.b - self.a;
		let baba = ba.dot(ba);
		if baba < 1e-12 {
			// Degenerate zero-length axis: behave as a sphere of the base radius.
			return (p - self.a).length() - self.ra;
		}
		let pa = p - self.a;
		let papa = pa.dot(pa);
		let paba = pa.dot(ba) / baba;
		let x = (papa - paba * paba * baba).max(0.0).sqrt();
		let cax = (x - if paba < 0.5 { self.ra } else { self.rb }).max(0.0);
		let cay = (paba - 0.5).abs() - 0.5;
		let k = rba * rba + baba;
		let f = ((rba * (x - self.ra) + paba * baba) / k).clamp(0.0, 1.0);
		let cbx = x - self.ra - f * rba;
		let cby = paba - f;
		let s = if cbx < 0.0 && cay < 0.0 { -1.0 } else { 1.0 };
		s * (cax * cax + cay * cay * baba).min(cbx * cbx + cby * cby * baba).sqrt()
	}
	fn distance64(&self, p: DVec3) -> f64 {
		let rba = (self.rb - self.ra) as f64;
		let ba = self.b.as_dvec3() - self.a.as_dvec3();
		let baba = ba.dot(ba);
		if baba < 1e-12 {
			return (p - self.a.as_dvec3()).length() - self.ra as f64;
		}
		let pa = p - self.a.as_dvec3();
		let papa = pa.dot(pa);
		let paba = pa.dot(ba) / baba;
		let x = (papa - paba * paba * baba).max(0.0).sqrt();
		let cax = (x - if paba < 0.5 { self.ra as f64 } else { self.rb as f64 }).max(0.0);
		let cay = (paba - 0.5).abs() - 0.5;
		let k = rba * rba + baba;
		let f = ((rba * (x - self.ra as f64) + paba * baba) / k).clamp(0.0, 1.0);
		let cbx = x - self.ra as f64 - f * rba;
		let cby = paba - f;
		let s = if cbx < 0.0 && cay < 0.0 { -1.0 } else { 1.0 };
		s * (cax * cax + cay * cay * baba).min(cbx * cbx + cby * cby * baba).sqrt()
	}
	fn bounds(&self) -> Aabb {
		Aabb::from_points(&[self.a, self.b]).pad(self.ra.max(self.rb))
	}
}

/// A half-space (oriented plane). Inside is where the signed distance is negative.
///
/// Unbounded on its own — intended for use inside CSG intersections, where the
/// other operand provides the bound. [`Plane::bounds`] returns a large box.
#[derive(Clone, Copy, Debug)]
pub struct Plane {
	pub normal: Vec3,
	pub offset: f32,
}

impl Plane {
	/// Plane through `point` with outward `normal` (auto-normalized).
	pub fn new(point: Vec3, normal: Vec3) -> Self {
		let n = normal.normalize_or_zero();
		Self { normal: n, offset: point.dot(n) }
	}
}

impl Sdf for Plane {
	fn distance(&self, p: Vec3) -> f32 {
		p.dot(self.normal) - self.offset
	}
	fn distance64(&self, p: DVec3) -> f64 {
		p.dot(self.normal.as_dvec3()) - self.offset as f64
	}
	fn bounds(&self) -> Aabb {
		// A half-space is genuinely unbounded. Reporting an infinite box keeps
		// the contract honest and composes correctly: intersecting it with a
		// finite operand yields that operand's (finite) bound, while meshing a
		// bare plane is rejected by the mesher's non-finite-domain guard.
		Aabb::new(Vec3::splat(f32::NEG_INFINITY), Vec3::splat(f32::INFINITY))
	}
	fn gradient(&self, _p: Vec3) -> Vec3 {
		self.normal
	}
}

/// A torus with the given `center`, ring `axis`, major radius and minor radius.
#[derive(Clone, Copy, Debug)]
pub struct Torus {
	pub center: Vec3,
	pub axis: Vec3,
	pub major: f32,
	pub minor: f32,
}

impl Torus {
	pub fn new(center: Vec3, axis: Vec3, major: f32, minor: f32) -> Self {
		Self { center, axis, major, minor }
	}
}

impl Sdf for Torus {
	fn distance(&self, p: Vec3) -> f32 {
		let n = self.axis.normalize_or_zero();
		let pl = p - self.center;
		let axial = pl.dot(n);
		let radial = (pl - n * axial).length();
		let q = Vec2::new(radial - self.major, axial);
		q.length() - self.minor
	}
	fn distance64(&self, p: DVec3) -> f64 {
		let n = self.axis.as_dvec3().normalize_or_zero();
		let pl = p - self.center.as_dvec3();
		let axial = pl.dot(n);
		let radial = (pl - n * axial).length();
		let q = DVec2::new(radial - self.major as f64, axial);
		q.length() - self.minor as f64
	}
	fn bounds(&self) -> Aabb {
		Aabb::from_center_half_extent(self.center, Vec3::splat(self.major + self.minor))
	}
}

/// A capsule (line segment `a`→`b` swept by a sphere of `radius`).
#[derive(Clone, Copy, Debug)]
pub struct Capsule {
	pub a: Vec3,
	pub b: Vec3,
	pub radius: f32,
}

impl Capsule {
	pub fn new(a: Vec3, b: Vec3, radius: f32) -> Self {
		Self { a, b, radius }
	}
}

impl Sdf for Capsule {
	fn distance(&self, p: Vec3) -> f32 {
		let pa = p - self.a;
		let ba = self.b - self.a;
		let baba = ba.dot(ba);
		if baba < 1e-12 {
			return pa.length() - self.radius; // degenerate: a sphere at `a`
		}
		let h = (pa.dot(ba) / baba).clamp(0.0, 1.0);
		(pa - ba * h).length() - self.radius
	}
	fn distance64(&self, p: DVec3) -> f64 {
		let pa = p - self.a.as_dvec3();
		let ba = self.b.as_dvec3() - self.a.as_dvec3();
		let baba = ba.dot(ba);
		if baba < 1e-12 {
			return pa.length() - self.radius as f64;
		}
		let h = (pa.dot(ba) / baba).clamp(0.0, 1.0);
		(pa - ba * h).length() - self.radius as f64
	}
	fn bounds(&self) -> Aabb {
		Aabb::from_points(&[self.a, self.b]).pad(self.radius)
	}
}

/// A gyroid TPMS shell, bounded to `region`.
///
/// This is an *implicit field*, not a true Euclidean SDF — the value is
/// normalized only roughly. It is meant to be intersected with a solid region
/// (e.g. a [`Cuboid`]) for a bounded lattice. `scale` sets the cell frequency;
/// `thickness` the half-wall thickness. Because it is a bound, wrap it with
/// [`Node::primitive_bound`](crate::Node::primitive_bound) (not `primitive`) so
/// a downstream `offset`/`shell` is honestly flagged approximate.
#[derive(Clone, Copy, Debug)]
pub struct Gyroid {
	pub region: Aabb,
	pub scale: f32,
	pub thickness: f32,
}

impl Gyroid {
	pub fn new(region: Aabb, scale: f32, thickness: f32) -> Self {
		Self { region, scale, thickness }
	}
}

impl Sdf for Gyroid {
	fn distance(&self, p: Vec3) -> f32 {
		// A zero (or near-zero) frequency has no surface; treat as a constant field
		// rather than dividing by zero and poisoning the grid with NaN.
		if self.scale.abs() < 1e-12 {
			return -self.thickness;
		}
		let q = p * self.scale;
		let g = q.x.sin() * q.y.cos() + q.y.sin() * q.z.cos() + q.z.sin() * q.x.cos();
		// Metric normalization by the field frequency, then a shell. The sine-sum's
		// gradient magnitude reaches √3·scale, so dividing by scale alone leaves a
		// √3-Lipschitz field — enough to defeat the narrow-band mesher's
		// Lipschitz-safe block pruning (a sampled value can overstate the true
		// distance and a block containing surface gets skipped). The extra /√3
		// guarantees |∇d| ≤ 1; the ZERO SET — and therefore the meshed geometry —
		// is exactly the same as before, only the field's slope changes.
		(g.abs() / self.scale - self.thickness) / 3.0_f32.sqrt()
	}
	fn distance64(&self, p: DVec3) -> f64 {
		if (self.scale as f64).abs() < 1e-12 {
			return -self.thickness as f64;
		}
		let q = p * self.scale as f64;
		let g = q.x.sin() * q.y.cos() + q.y.sin() * q.z.cos() + q.z.sin() * q.x.cos();
		(g.abs() / self.scale as f64 - self.thickness as f64) / 3.0_f64.sqrt()
	}
	fn bounds(&self) -> Aabb {
		self.region
	}
}

/// Triply-periodic minimal-surface (TPMS) lattice families for [`Tpms`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TpmsKind {
	/// `sin x·cos y + sin y·cos z + sin z·cos x` — the gyroid (chiral, isotropic).
	Gyroid,
	/// `cos x + cos y + cos z` — Schwarz primitive (cubic cells, big round pores).
	SchwarzP,
	/// Schwarz diamond — `sxsysz + sxcycz + cxsycz + cxcysz` (interlocking tetrapods).
	Diamond,
	/// Neovius — `3(cos x+cos y+cos z)+4·cos x cos y cos z` (thick nodes, small
	/// windows; stiff, low-porosity).
	Neovius,
	/// Schoen I-WP — `2(cxcy+cycz+czcx)−(cos 2x+cos 2y+cos 2z)` ("wrapped package";
	/// one bulky labyrinth of star-shaped cells).
	SchoenIwp,
	/// Fischer–Koch S — `cos 2x·sin y·cos z + cos x·cos 2y·sin z + sin x·cos y·cos 2z`
	/// (fine, gently anisotropic cells; the shallowest field slope of the family).
	FischerKochS,
}

impl TpmsKind {
	/// The dimensionless TPMS field at scaled point `q` (its zero set is the surface).
	#[inline]
	fn field(self, q: Vec3) -> f32 {
		match self {
			TpmsKind::Gyroid => q.x.sin() * q.y.cos() + q.y.sin() * q.z.cos() + q.z.sin() * q.x.cos(),
			TpmsKind::SchwarzP => q.x.cos() + q.y.cos() + q.z.cos(),
			TpmsKind::Diamond => {
				let (sx, sy, sz) = (q.x.sin(), q.y.sin(), q.z.sin());
				let (cx, cy, cz) = (q.x.cos(), q.y.cos(), q.z.cos());
				sx * sy * sz + sx * cy * cz + cx * sy * cz + cx * cy * sz
			}
			TpmsKind::Neovius => {
				let (cx, cy, cz) = (q.x.cos(), q.y.cos(), q.z.cos());
				3.0 * (cx + cy + cz) + 4.0 * cx * cy * cz
			}
			TpmsKind::SchoenIwp => {
				let (cx, cy, cz) = (q.x.cos(), q.y.cos(), q.z.cos());
				2.0 * (cx * cy + cy * cz + cz * cx) - ((2.0 * q.x).cos() + (2.0 * q.y).cos() + (2.0 * q.z).cos())
			}
			TpmsKind::FischerKochS => {
				let (sx, sy, sz) = (q.x.sin(), q.y.sin(), q.z.sin());
				let (cy, cz) = (q.y.cos(), q.z.cos());
				(2.0 * q.x).cos() * sy * cz + q.x.cos() * (2.0 * q.y).cos() * sz + sx * cy * (2.0 * q.z).cos()
			}
		}
	}

	#[inline]
	fn field64(self, q: DVec3) -> f64 {
		match self {
			TpmsKind::Gyroid => q.x.sin() * q.y.cos() + q.y.sin() * q.z.cos() + q.z.sin() * q.x.cos(),
			TpmsKind::SchwarzP => q.x.cos() + q.y.cos() + q.z.cos(),
			TpmsKind::Diamond => {
				let (sx, sy, sz) = (q.x.sin(), q.y.sin(), q.z.sin());
				let (cx, cy, cz) = (q.x.cos(), q.y.cos(), q.z.cos());
				sx * sy * sz + sx * cy * cz + cx * sy * cz + cx * cy * sz
			}
			TpmsKind::Neovius => {
				let (cx, cy, cz) = (q.x.cos(), q.y.cos(), q.z.cos());
				3.0 * (cx + cy + cz) + 4.0 * cx * cy * cz
			}
			TpmsKind::SchoenIwp => {
				let (cx, cy, cz) = (q.x.cos(), q.y.cos(), q.z.cos());
				2.0 * (cx * cy + cy * cz + cz * cx) - ((2.0 * q.x).cos() + (2.0 * q.y).cos() + (2.0 * q.z).cos())
			}
			TpmsKind::FischerKochS => {
				let (sx, sy, sz) = (q.x.sin(), q.y.sin(), q.z.sin());
				let (cy, cz) = (q.y.cos(), q.z.cos());
				(2.0 * q.x).cos() * sy * cz + q.x.cos() * (2.0 * q.y).cos() * sz + sx * cy * (2.0 * q.z).cos()
			}
		}
	}

	/// Upper bound on `|∇field|` in scaled coordinates; the field is divided by it
	/// so the SDF stays ≤ 1-Lipschitz (required by narrow-band block pruning).
	/// Dividing by a constant does not move the zero set, so this only affects
	/// pruning safety, never the geometry. Values are the true field-gradient
	/// maxima (analytic where clean, else numerically pinned on a 200³ cell grid):
	/// Gyroid/Schwarz-P `√3`, Diamond/Fischer-Koch `√6`, Neovius `7`, I-WP `3√3`.
	#[inline]
	fn lipschitz(self) -> f32 {
		match self {
			TpmsKind::Diamond | TpmsKind::FischerKochS => 6.0_f32.sqrt(),
			TpmsKind::Neovius => 7.0,
			TpmsKind::SchoenIwp => 3.0 * 3.0_f32.sqrt(),
			_ => 3.0_f32.sqrt(),
		}
	}
}

/// A TPMS lattice ([`TpmsKind`]) as an [`Sdf`], in one of two modes.
///
/// - **Network** ([`Tpms::network`]): the SOLID single labyrinth `field < level`.
///   `level = 0` fills ≈ 50%; **more negative thins the network toward its
///   skeletal graph — light yet still ONE connected piece**, because connectivity
///   is volumetric (a 3-D labyrinth), not a thin surface that pinches apart. This
///   is the light-and-connected organic infill (vs. a strut [`crate::BeamLattice`]).
/// - **Sheet** ([`Tpms::sheet`]): the thickened minimal surface `|field| < t`, the
///   classic double-wall TPMS shell. (For the gyroid this matches [`Gyroid`].)
///
/// `cell` is the unit-cell period in model units. The trig field is periodic
/// everywhere, so [`bounds`](Sdf::bounds) returns `region` and you should
/// intersect the lattice with a shroud solid to bound it. Like [`Gyroid`] the
/// normalized trig field is a distance BOUND, so wrap it with
/// [`Node::primitive_bound`](crate::Node::primitive_bound) when a downstream
/// `offset`/`shell` needs to know.
#[derive(Clone, Copy, Debug)]
pub struct Tpms {
	pub region: Aabb,
	pub kind: TpmsKind,
	/// Spatial frequency `2π / cell`.
	pub scale: f32,
	/// Network iso-level, or sheet half-thickness — see the mode.
	pub level: f32,
	/// `true` = thickened sheet; `false` = solid network.
	pub sheet: bool,
}

impl Tpms {
	/// Solid single-labyrinth network of `kind`. `level = 0` ≈ 50% solid; negative
	/// thins it (lighter, stays connected); positive thickens it.
	pub fn network(region: Aabb, kind: TpmsKind, cell: f32, level: f32) -> Self {
		Self { region, kind, scale: std::f32::consts::TAU / cell, level, sheet: false }
	}

	/// Thickened sheet of `kind` (the minimal surface grown by `thickness`).
	pub fn sheet(region: Aabb, kind: TpmsKind, cell: f32, thickness: f32) -> Self {
		Self { region, kind, scale: std::f32::consts::TAU / cell, level: thickness, sheet: true }
	}
}

impl Sdf for Tpms {
	fn distance(&self, p: Vec3) -> f32 {
		if self.scale.abs() < 1e-12 {
			return -self.level.abs();
		}
		let f = self.kind.field(p * self.scale);
		let raw = if self.sheet { f.abs() / self.scale - self.level } else { f / self.scale - self.level };
		raw / self.kind.lipschitz()
	}

	fn distance64(&self, p: DVec3) -> f64 {
		let s = self.scale as f64;
		if s.abs() < 1e-12 {
			return -(self.level.abs() as f64);
		}
		let f = self.kind.field64(p * s);
		let raw = if self.sheet { f.abs() / s - self.level as f64 } else { f / s - self.level as f64 };
		raw / self.kind.lipschitz() as f64
	}

	fn bounds(&self) -> Aabb {
		self.region
	}
}
