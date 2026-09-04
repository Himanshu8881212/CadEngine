//! Cross-section properties must match closed-form values on analytic bodies —
//! the foundation under any strength estimate built on them.
use kernel_core::math::Vec3;
use kernel_core::Mesh;
use std::f64::consts::{PI, TAU};

fn tube(ro: f64, ri: f64, h: f64, seg: usize) -> Mesh {
	let mut m = Mesh::default();
	let mut quad = |a: Vec3, b: Vec3, c: Vec3, d: Vec3| {
		let base = m.positions.len() as u32;
		m.positions.extend_from_slice(&[a, b, c, d]);
		m.indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
	};
	for k in 0..seg {
		let (a0, a1) = (TAU * k as f64 / seg as f64, TAU * (k + 1) as f64 / seg as f64);
		let (c0, s0, c1, s1) = (a0.cos() as f32, a0.sin() as f32, a1.cos() as f32, a1.sin() as f32);
		let (ro, ri, h) = (ro as f32, ri as f32, h as f32);
		// outer wall (outward), inner wall (inward), end rings
		quad(
			Vec3::new(ro * c0, ro * s0, 0.0),
			Vec3::new(ro * c1, ro * s1, 0.0),
			Vec3::new(ro * c1, ro * s1, h),
			Vec3::new(ro * c0, ro * s0, h),
		);
		if ri > 0.0 {
			quad(
				Vec3::new(ri * c1, ri * s1, 0.0),
				Vec3::new(ri * c0, ri * s0, 0.0),
				Vec3::new(ri * c0, ri * s0, h),
				Vec3::new(ri * c1, ri * s1, h),
			);
			quad(
				Vec3::new(ro * c1, ro * s1, 0.0),
				Vec3::new(ro * c0, ro * s0, 0.0),
				Vec3::new(ri * c0, ri * s0, 0.0),
				Vec3::new(ri * c1, ri * s1, 0.0),
			);
			quad(
				Vec3::new(ro * c0, ro * s0, h),
				Vec3::new(ro * c1, ro * s1, h),
				Vec3::new(ri * c1, ri * s1, h),
				Vec3::new(ri * c0, ri * s0, h),
			);
		} else {
			quad(Vec3::new(ro * c1, ro * s1, 0.0), Vec3::new(ro * c0, ro * s0, 0.0), Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 0.0));
			quad(Vec3::new(ro * c0, ro * s0, h), Vec3::new(ro * c1, ro * s1, h), Vec3::new(0.0, 0.0, h), Vec3::new(0.0, 0.0, h));
		}
	}
	m
}

#[test]
fn section_properties_match_closed_forms() {
	let seg = 256;
	let solid = tube(10.0, 0.0, 20.0, seg);
	let s = solid.section_properties(Vec3::new(0.0, 0.0, 10.0), Vec3::Z).expect("solid section");
	let tube_m = tube(10.0, 6.0, 20.0, seg);
	let t = tube_m.section_properties(Vec3::new(0.0, 0.0, 10.0), Vec3::Z).expect("tube section");
	// closed forms (chordal polygon slightly under the circle — 0.5 % at 256 seg)
	let (a_c, i_c) = (PI * 100.0, PI * 1e4 / 4.0);
	let (a_t, i_t) = (PI * (100.0 - 36.0), PI * (1e4 - 1296.0) / 4.0);
	assert!(
		(s.area - a_c).abs() / a_c < 0.005
			&& (s.i_uu - i_c).abs() / i_c < 0.01
			&& (s.i_vv - i_c).abs() / i_c < 0.01
			&& (s.c_max - 10.0).abs() < 0.05
			&& (t.area - a_t).abs() / a_t < 0.005
			&& (t.i_uu - i_t).abs() / i_t < 0.01,
		"section properties vs closed forms: solid A {:.2} (want {a_c:.2}) I {:.1} (want {i_c:.1}) c {:.3}; tube A {:.2} (want {a_t:.2}) I {:.1} (want {i_t:.1})",
		s.area,
		s.i_uu,
		s.c_max,
		t.area,
		t.i_uu
	);
}
