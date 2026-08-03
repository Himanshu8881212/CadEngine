//! Regression: `Aabb::ray_hits` must not false-miss a grazing axis-parallel ray
//! whose origin lies exactly on a slab face (the `0*inf=NaN` poisoning bug).

use kernel_core::{Aabb, Ray, Vec3};

#[test]
fn ray_hits_grazing_axis_parallel_ray_on_box_face() {
	let b = Aabb::new(Vec3::ZERO, Vec3::splat(10.0));
	// Ray lying on the x=min face, travelling -z (parallel to the x and y slabs):
	// the old test computed 0*inf=NaN on the x axis and rejected this real hit.
	let graze = Ray::new(Vec3::new(0.0, 5.0, 5.0), Vec3::new(0.0, 0.0, -1.0));
	// A clean interior hit, and a ray parallel-and-outside the x slab (true miss).
	let inside = Ray::new(Vec3::new(5.0, 5.0, 20.0), Vec3::new(0.0, 0.0, -1.0));
	let miss = Ray::new(Vec3::new(-1.0, 5.0, 5.0), Vec3::new(0.0, 0.0, -1.0));
	let (g, i, m) = (b.ray_hits(graze, 100.0), b.ray_hits(inside, 100.0), b.ray_hits(miss, 100.0));
	assert!(
		g && i && !m,
		"grazing axis-parallel ray on a box face must hit, interior must hit, parallel-outside must miss: graze={g} inside={i} miss={m}"
	);
}
