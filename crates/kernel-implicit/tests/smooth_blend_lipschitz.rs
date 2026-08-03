//! The smooth blends must stay <=1-Lipschitz — the contract the narrow-band
//! mesher relies on to seed safely (a field with |grad|>1 can let the coarse
//! seeding step over the surface). The standard polynomial smooth-min overshoots
//! 1 in the blend region; this pins that the engine's variant does NOT, so smooth
//! / organic shapes are safe to narrow-band mesh without a redistance pass.

use kernel_implicit::{Node, Sdf, Sphere, Vec3};

fn max_gradient_magnitude(n: &Node) -> f32 {
	let mut m = 0.0f32;
	let r = 24i32; // +/- 9.6 mm at 0.4 spacing — covers the blend region between the spheres
	for i in -r..=r {
		for j in -r..=r {
			for k in -r..=r {
				let g = n.gradient(Vec3::new(i as f32 * 0.4, j as f32 * 0.4, k as f32 * 0.4)).length();
				if g.is_finite() && g > m {
					m = g;
				}
			}
		}
	}
	m
}

#[test]
fn smooth_blends_stay_one_lipschitz() {
	let a = || Node::primitive(Sphere::new(Vec3::ZERO, 5.0));
	let b = || Node::primitive(Sphere::new(Vec3::new(6.0, 0.0, 0.0), 5.0));
	// A non-Lipschitz smooth-min would peak well above 1 in the blend; allow only a
	// tiny finite-precision margin.
	for k in [0.5f32, 1.0, 2.0, 4.0] {
		let g = max_gradient_magnitude(&a().smooth_union(b(), k));
		assert!(g <= 1.01, "smooth_union(k={k}) must stay <=1-Lipschitz (narrow-band safety): max|grad|={g}");
	}
	let gi = max_gradient_magnitude(&a().smooth_intersection(b(), 2.0));
	let gd = max_gradient_magnitude(&a().smooth_difference(b(), 2.0));
	assert!(
		gi <= 1.01 && gd <= 1.01,
		"smooth_intersection / smooth_difference must stay <=1-Lipschitz: inter={gi} diff={gd}"
	);
}
