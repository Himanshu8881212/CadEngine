// Copyright (c) LMCAD. Licensed under the MIT License.

//! The unifying [`Sdf`] trait — the common interface between the B-rep,
//! CSG, and voxel representations.
//!
//! Everything that can report a signed distance, a bound, and a surface
//! normal can be meshed and combined. This single abstraction *is* the
//! bridge of the hybrid kernel.

use crate::math::{Aabb, DVec3, Vec3};

/// A signed distance function over `f32` world space.
///
/// Sign convention: **negative inside, positive outside**, with the value
/// equal to the (signed) Euclidean distance to the surface. For an exact
/// SDF the gradient is the unit outward normal.
///
/// The supertrait bounds (`Send + Sync`) let meshers sample an SDF in
/// parallel across threads.
pub trait Sdf: Send + Sync {
	/// Signed distance from `p` to the surface (negative inside).
	fn distance(&self, p: Vec3) -> f32;

	/// Double-precision signed distance. Defaults to the `f32` evaluation widened
	/// to `f64`; analytic primitives and the CSG combinators override it to evaluate
	/// in `f64` throughout. This avoids the catastrophic cancellation of `large −
	/// large` and the quantization of the query point, so **point classification and
	/// proximity stay accurate on large parts** (metre-scale models with millimetre
	/// features) where the `f32` path loses the feature in rounding.
	fn distance64(&self, p: DVec3) -> f64 {
		self.distance(p.as_vec3()) as f64
	}

	/// A bound that fully contains the surface. May be conservative.
	fn bounds(&self) -> Aabb;

	/// Unit outward normal at `p`.
	///
	/// Defaults to a central-difference of [`Sdf::distance`]; analytic
	/// primitives should override this with a closed-form gradient.
	fn gradient(&self, p: Vec3) -> Vec3 {
		central_difference(self, p, 1e-4)
	}
}

/// Estimate a unit gradient (outward normal) via central differences.
///
/// `eps` is the half-step (in world units). The result is normalized so it
/// is usable directly as a shading / Hermite normal even for voxel grids
/// where the raw gradient magnitude is not exactly one.
pub fn central_difference<S: Sdf + ?Sized>(sdf: &S, p: Vec3, eps: f32) -> Vec3 {
	let dx = sdf.distance(p + Vec3::new(eps, 0.0, 0.0)) - sdf.distance(p - Vec3::new(eps, 0.0, 0.0));
	let dy = sdf.distance(p + Vec3::new(0.0, eps, 0.0)) - sdf.distance(p - Vec3::new(0.0, eps, 0.0));
	let dz = sdf.distance(p + Vec3::new(0.0, 0.0, eps)) - sdf.distance(p - Vec3::new(0.0, 0.0, eps));
	let g = Vec3::new(dx, dy, dz);
	let len = g.length();
	if len > 1e-12 {
		g / len
	} else {
		Vec3::Z
	}
}

// --- Blanket impls so references / boxes are themselves `Sdf` ----------------

impl<S: Sdf + ?Sized> Sdf for &S {
	fn distance(&self, p: Vec3) -> f32 {
		(**self).distance(p)
	}
	fn distance64(&self, p: DVec3) -> f64 {
		(**self).distance64(p)
	}
	fn bounds(&self) -> Aabb {
		(**self).bounds()
	}
	fn gradient(&self, p: Vec3) -> Vec3 {
		(**self).gradient(p)
	}
}

impl<S: Sdf + ?Sized> Sdf for Box<S> {
	fn distance(&self, p: Vec3) -> f32 {
		(**self).distance(p)
	}
	fn distance64(&self, p: DVec3) -> f64 {
		(**self).distance64(p)
	}
	fn bounds(&self) -> Aabb {
		(**self).bounds()
	}
	fn gradient(&self, p: Vec3) -> Vec3 {
		(**self).gradient(p)
	}
}
