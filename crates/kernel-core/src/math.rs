// Copyright (c) LMCAD. Licensed under the MIT License.

//! Core math types for the kernel.
//!
//! We re-export [`glam`] so every crate shares one linear-algebra vocabulary.
//! Convention (per the engineering spec):
//! - `f32` / [`Vec3`] for the implicit / voxel side (memory + speed).
//! - `f64` / [`DVec3`] for the exact B-rep side (precision).

pub use glam::{Affine3A, DAffine3, DMat3, DMat4, DQuat, DVec2, DVec3, DVec4, Mat3, Mat4, Quat, Vec2, Vec3, Vec3A, Vec4};

/// Default linear tolerance (mm) for geometric comparisons.
pub const EPSILON: f32 = 1e-5;

/// Tolerance used for SDF zero-crossing tests along an edge.
pub const SURFACE_EPSILON: f32 = 1e-6;

/// An axis-aligned bounding box in `f32` world space.
///
/// An [`Aabb::empty`] box has `min = +inf`, `max = -inf` so that [`Aabb::union`]
/// and [`Aabb::expand_point`] behave as monoid identities.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Aabb {
	pub min: Vec3,
	pub max: Vec3,
}

impl Aabb {
	/// Construct from explicit corners (no validation).
	pub fn new(min: Vec3, max: Vec3) -> Self {
		Self { min, max }
	}

	/// The identity box for union/expand: inverted infinite extents.
	pub fn empty() -> Self {
		Self { min: Vec3::splat(f32::INFINITY), max: Vec3::splat(f32::NEG_INFINITY) }
	}

	/// A box centred at `center` with the given half extent.
	pub fn from_center_half_extent(center: Vec3, half_extent: Vec3) -> Self {
		Self { min: center - half_extent, max: center + half_extent }
	}

	/// Tightest box containing all `points`.
	pub fn from_points(points: &[Vec3]) -> Self {
		let mut b = Self::empty();
		for &p in points {
			b = b.expand_point(p);
		}
		b
	}

	/// Grow to include `p`.
	pub fn expand_point(self, p: Vec3) -> Self {
		Self { min: self.min.min(p), max: self.max.max(p) }
	}

	/// Smallest box containing both `self` and `other`.
	pub fn union(self, other: Self) -> Self {
		Self { min: self.min.min(other.min), max: self.max.max(other.max) }
	}

	/// Intersection box (may be invalid/empty if the boxes do not overlap).
	pub fn intersection(self, other: Self) -> Self {
		Self { min: self.min.max(other.min), max: self.max.min(other.max) }
	}

	/// Uniformly pad the box outward by `margin` on every side.
	pub fn pad(self, margin: f32) -> Self {
		Self { min: self.min - Vec3::splat(margin), max: self.max + Vec3::splat(margin) }
	}

	/// Centre point.
	pub fn center(self) -> Vec3 {
		(self.min + self.max) * 0.5
	}

	/// Full extent (`max - min`).
	pub fn size(self) -> Vec3 {
		self.max - self.min
	}

	/// Half extent (`size / 2`).
	pub fn half_extent(self) -> Vec3 {
		self.size() * 0.5
	}

	/// Length of the box diagonal.
	pub fn diagonal(self) -> f32 {
		self.size().length()
	}

	/// True if `p` lies within (inclusive) the box.
	pub fn contains(self, p: Vec3) -> bool {
		p.cmpge(self.min).all() && p.cmple(self.max).all()
	}

	/// Surface area of the box.
	pub fn surface_area(self) -> f32 {
		let s = self.size();
		2.0 * (s.x * s.y + s.y * s.z + s.z * s.x)
	}

	/// Volume of the box.
	pub fn volume(self) -> f32 {
		let s = self.size();
		s.x * s.y * s.z
	}

	/// True if `min <= max` on every axis.
	pub fn is_valid(self) -> bool {
		self.min.cmple(self.max).all()
	}

	/// The eight corners, ordered by the bit pattern `(x | y<<1 | z<<2)`.
	pub fn corners(self) -> [Vec3; 8] {
		let (lo, hi) = (self.min, self.max);
		[
			Vec3::new(lo.x, lo.y, lo.z),
			Vec3::new(hi.x, lo.y, lo.z),
			Vec3::new(lo.x, hi.y, lo.z),
			Vec3::new(hi.x, hi.y, lo.z),
			Vec3::new(lo.x, lo.y, hi.z),
			Vec3::new(hi.x, lo.y, hi.z),
			Vec3::new(lo.x, hi.y, hi.z),
			Vec3::new(hi.x, hi.y, hi.z),
		]
	}

	/// Squared distance from a point to the box (`0` when the point is inside).
	pub fn distance_squared(self, p: Vec3) -> f32 {
		let q = p.clamp(self.min, self.max);
		(p - q).length_squared()
	}

	/// Squared distance between two boxes (`0` when they overlap or touch).
	pub fn distance_squared_box(self, other: Aabb) -> f32 {
		let dx = (self.min.x - other.max.x).max(other.min.x - self.max.x).max(0.0);
		let dy = (self.min.y - other.max.y).max(other.min.y - self.max.y).max(0.0);
		let dz = (self.min.z - other.max.z).max(other.min.z - self.max.z).max(0.0);
		dx * dx + dy * dy + dz * dz
	}

	/// Whether `ray` enters the box at a parameter in `[0, tmax]` (slab method).
	/// Conservative: a grazing ray may report `true`, which only costs extra exact
	/// triangle tests downstream — it never reports a false miss for a real hit.
	///
	/// NaN-robust per axis: a direction component of (near-)zero makes `1/dir`
	/// overflow to `±inf`, and a ray whose origin lies exactly on that slab face
	/// would compute `0·inf = NaN` and poison a vectorised `min`/`max` reduction
	/// (a real hit rejected). Such an axis is handled as "parallel to the slab":
	/// it constrains nothing if the origin is inside the slab, else it is a miss.
	pub fn ray_hits(self, ray: Ray, tmax: f32) -> bool {
		let (org, dir) = (ray.origin.to_array(), ray.dir.to_array());
		let (lo, hi) = (self.min.to_array(), self.max.to_array());
		let mut enter = 0.0f32;
		let mut exit = tmax;
		for a in 0..3 {
			let inv = 1.0 / dir[a];
			if inv.is_finite() {
				let mut t0 = (lo[a] - org[a]) * inv;
				let mut t1 = (hi[a] - org[a]) * inv;
				if t0 > t1 {
					core::mem::swap(&mut t0, &mut t1);
				}
				enter = enter.max(t0);
				exit = exit.min(t1);
				if enter > exit {
					return false;
				}
			} else if org[a] < lo[a] || org[a] > hi[a] {
				return false;
			}
		}
		enter <= exit
	}
}

/// An oriented bounding box: a box centered at `center`, with three unit `axes`
/// (right-handed, as the columns of the rotation) and the `half_extents` measured
/// along them. Tighter than an [`Aabb`] for shapes not aligned to the world axes.
/// Held in `f64` to match the precision of the inertia frame it is built from.
#[derive(Clone, Copy, Debug)]
pub struct Obb {
	pub center: DVec3,
	pub axes: DMat3,
	pub half_extents: DVec3,
}

impl Obb {
	/// The enclosed volume (`8·∏ half_extents`).
	pub fn volume(self) -> f64 {
		8.0 * self.half_extents.x * self.half_extents.y * self.half_extents.z
	}

	/// The eight corner points in world space.
	pub fn corners(self) -> [DVec3; 8] {
		let (e0, e1, e2) = (self.axes.x_axis, self.axes.y_axis, self.axes.z_axis);
		let h = self.half_extents;
		let mut c = [DVec3::ZERO; 8];
		let mut k = 0;
		for sx in [-1.0, 1.0] {
			for sy in [-1.0, 1.0] {
				for sz in [-1.0, 1.0] {
					c[k] = self.center + e0 * (sx * h.x) + e1 * (sy * h.y) + e2 * (sz * h.z);
					k += 1;
				}
			}
		}
		c
	}

	/// Whether a world-space point lies inside the box (within a small tolerance).
	pub fn contains(self, p: DVec3) -> bool {
		let d = p - self.center;
		let eps = 1e-7 * self.half_extents.max_element().max(1.0);
		d.dot(self.axes.x_axis).abs() <= self.half_extents.x + eps
			&& d.dot(self.axes.y_axis).abs() <= self.half_extents.y + eps
			&& d.dot(self.axes.z_axis).abs() <= self.half_extents.z + eps
	}
}

/// A ray with a (not necessarily normalized) direction.
#[derive(Clone, Copy, Debug)]
pub struct Ray {
	pub origin: Vec3,
	pub dir: Vec3,
}

impl Ray {
	pub fn new(origin: Vec3, dir: Vec3) -> Self {
		Self { origin, dir }
	}

	/// Point at parameter `t` along the ray.
	pub fn at(self, t: f32) -> Vec3 {
		self.origin + self.dir * t
	}
}
