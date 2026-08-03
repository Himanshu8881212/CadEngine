// Copyright (c) LMCAD. Licensed under the MIT License.

//! Acceptance for the surface-texture displacement fields (`texture.rs`) and
//! the single-stroke Hershey Simplex text field (`text.rs`).
//!
//! Every declared Lipschitz bound is *probed* here (sampled forward-difference
//! gradient must stay under the declaration — the same heuristic contract as
//! `kernel-api`'s `probe_lipschitz` and `tests/tpms.rs`), the knurled-cylinder
//! volume is checked against the analytic displacement window, the hashed
//! textures are checked bit-deterministic, and the text field is checked
//! against the font's own advance metrics plus a real engrave.

use kernel_implicit::text::{text_advance, text_field};
use kernel_implicit::texture::{displaced, Displaced, Texture};
use kernel_implicit::{
	check_mesh, manifold_dual_contour, surface_nets, surface_nets_narrowband, Cuboid, Cylinder, Node, Resolution,
	Sdf, Vec3,
};

/// Sampled `sup |∇f|` over an `n³` lattice on `[lo, hi]` (forward differences
/// with step `h`) — the tests' probe of a declared Lipschitz bound. A
/// heuristic lower bound on the true supremum, like `probe_lipschitz`: it
/// reliably catches an under-declared bound, it is not a certificate.
fn probe_max_grad(f: impl Fn(Vec3) -> f32, lo: Vec3, hi: Vec3, n: usize, h: f32) -> f32 {
	let mut max_g = 0.0_f32;
	for i in 0..n {
		for j in 0..n {
			for k in 0..n {
				let t = Vec3::new(i as f32, j as f32, k as f32) / (n - 1) as f32;
				let p = lo + (hi - lo) * t;
				let d = f(p);
				let gx = (f(p + Vec3::new(h, 0.0, 0.0)) - d) / h;
				let gy = (f(p + Vec3::new(0.0, h, 0.0)) - d) / h;
				let gz = (f(p + Vec3::new(0.0, 0.0, h)) - d) / h;
				max_g = max_g.max((gx * gx + gy * gy + gz * gz).sqrt());
			}
		}
	}
	max_g
}

/// Sampled range of a texture field over an `n³` lattice on `[lo, hi]`.
fn probe_range(tex: &Texture, lo: Vec3, hi: Vec3, n: usize) -> (f32, f32) {
	let (mut t_min, mut t_max) = (f32::INFINITY, f32::NEG_INFINITY);
	for i in 0..n {
		for j in 0..n {
			for k in 0..n {
				let t = Vec3::new(i as f32, j as f32, k as f32) / (n - 1) as f32;
				let v = tex.value(lo + (hi - lo) * t);
				t_min = t_min.min(v);
				t_max = t_max.max(v);
			}
		}
	}
	(t_min, t_max)
}

#[test]
fn knurled_cylinder_volume_window_lipschitz_and_narrowband_soundness() {
	// (a) A Ø16 × 30 post with a 2 mm crossed knurl, amplitude 0.4 mm.
	let (r, len, amp) = (8.0_f32, 30.0_f32, 0.4_f32);
	let tex = Texture::Knurl { pitch: 2.0, depth_frac: 1.0 };
	let l_t = tex.lipschitz(); // derived: depth_frac·π/pitch = π/2 ≈ 1.5708
	let cyl = || Cylinder::new(Vec3::ZERO, Vec3::new(0.0, 0.0, len), r);
	let declared = Displaced::new(cyl(), amp, tex).lipschitz_normalizer(); // 1 + 0.4·π/2 ≈ 1.6283
	let node = displaced(Node::primitive(cyl()), amp, tex);

	// Displacement is outward-only (t ≥ 0), mean ≈ amp/2, so the meshed volume
	// must land inside the analytic window base ± amp·lateral_area. The CLOSED
	// mesh comes from Manifold DC (the crate's manifold-guaranteed extractor —
	// plain surface nets leaves non-manifold edges on the knurl's egg-crate
	// saddles, exactly the artifact MDC exists to prevent).
	let base = std::f64::consts::PI * (r as f64).powi(2) * len as f64; // 6031.86
	let lateral = std::f64::consts::TAU * r as f64 * len as f64; // 1507.96
	let region = node.bounds().pad(0.6);
	let mdc = manifold_dual_contour(&node, region, Resolution::VoxelSize(0.2));
	let rep = check_mesh(&mdc);
	let vol = mdc.signed_volume();

	// The declared-bound probes: the emitted field is normalized to ≤ 1 (the
	// crate's narrow-band contract, divided by `declared`); the raw texture
	// field must respect its derived L_t.
	let grad_field = probe_max_grad(|p| node.distance(p), region.min, region.max, 24, 0.01);
	let grad_tex = probe_max_grad(|p| tex.value(p), Vec3::new(-8.0, -8.0, 0.0), Vec3::new(8.0, 8.0, 30.0), 24, 0.01);
	let (t_min, t_max) = probe_range(&tex, Vec3::new(-8.0, -8.0, 0.0), Vec3::new(8.0, 8.0, 30.0), 24);

	// End-to-end proof the normalization keeps narrow-band pruning sound: the
	// pruned surface-nets must agree with the dense surface-nets on the SAME
	// field (a pruned surface-bearing block would tear holes and shift volume).
	let sn_vol = surface_nets(&node, region, Resolution::VoxelSize(0.2)).signed_volume();
	let nb_vol = surface_nets_narrowband(&node, region, Resolution::VoxelSize(0.2)).signed_volume();

	// Measured (pinned 2026-07-29): MDC vol 6422.5 in window 5428.7..6635.0
	// (base 6031.9 + mean outward amp/2 over the whole skin); dense SN 6419.4
	// == narrow-band SN 6419.4; field max|∇| 0.755 (≤ 1 declared); texture
	// max|∇| 0.599 vs declared L_t 1.5708 (the tight constant is π/(√6·pitch)
	// = 0.641 via the gyroid identity in the Knurl docs — the declaration is
	// deliberately the simple conservative bound); t ∈ [0.2501, 0.7499]
	// (analytic [¼·depth_frac, ¾·depth_frac]).
	assert!(
		mdc.is_watertight()
			&& rep.boundary_edges == 0
			&& rep.non_manifold_edges == 0
			&& vol > base - amp as f64 * lateral
			&& vol < base + amp as f64 * lateral
			&& grad_field <= 1.05
			&& grad_tex <= 1.05 * l_t
			&& (0.0..=1.0).contains(&t_min)
			&& (0.0..=1.0).contains(&t_max)
			&& (nb_vol - sn_vol).abs() / sn_vol < 0.005
			&& (sn_vol - vol).abs() / vol < 0.02,
		"knurled cylinder: MDC watertight={} bnd={} nme={} vol={vol:.1} (window {:.1}..{:.1}, base {base:.1}) \
		 field max|grad|={grad_field:.3} (declared ≤1 after /{declared:.4}) \
		 texture max|grad|={grad_tex:.3} (declared L_t={l_t:.4}) t∈[{t_min:.3},{t_max:.3}] (contract [0,1]) \
		 narrow-band SN vol={nb_vol:.1} vs dense SN vol={sn_vol:.1} (must agree within 0.5%; MDC within 2%)",
		mdc.is_watertight(),
		rep.boundary_edges,
		rep.non_manifold_edges,
		base - amp as f64 * lateral,
		base + amp as f64 * lateral
	);
}

#[test]
fn stipple_field_honours_coverage_stays_in_cell_and_declared_lipschitz() {
	// Per-cell hashed domes: coverage fraction must track the parameter, the
	// field must stay in [0,1], and the sampled gradient must respect the
	// derived bound L_t = 8/(3√3·0.35·cell) ≈ 4.400/cell.
	let (cell, coverage) = (2.0_f32, 0.4_f32);
	let tex = Texture::Stipple { cell, coverage };
	let l_t = tex.lipschitz(); // ≈ 2.1998 at cell = 2

	// Dome centers are jittered ≤ 0.15·cell per axis with radius 0.35·cell, so
	// an occupied cell ALWAYS has t > 0 at its center (offset ≤ 0.26·cell) and
	// an empty cell has exactly 0 (domes never leave their own cell): counting
	// positive cell-center samples IS the occupancy fraction.
	let n = 12_i64; // 12³ = 1728 cells → binomial σ ≈ 0.012
	let mut occupied = 0_u32;
	for i in -n / 2..n / 2 {
		for j in -n / 2..n / 2 {
			for k in -n / 2..n / 2 {
				let center = Vec3::new(i as f32 + 0.5, j as f32 + 0.5, k as f32 + 0.5) * cell;
				if tex.value(center) > 0.0 {
					occupied += 1;
				}
			}
		}
	}
	let frac = occupied as f32 / (n * n * n) as f32;

	let lo = Vec3::splat(-6.0);
	let hi = Vec3::splat(6.0);
	let grad = probe_max_grad(|p| tex.value(p), lo, hi, 40, 0.01);
	let (t_min, t_max) = probe_range(&tex, lo, hi, 40);

	// Measured (pinned 2026-07-29): frac 0.4028 (want 0.40), max|∇| 2.227 =
	// 1.013·L_t (forward-difference overshoot inside the 1.05 slack — the
	// derived dome bound is essentially TIGHT), t ∈ [0.000, 0.9973].
	assert!(
		(frac - coverage).abs() < 0.06
			&& grad <= 1.05 * l_t
			&& t_min == 0.0
			&& t_max > 0.5
			&& t_max <= 1.0,
		"stipple: occupied fraction {frac:.3} (want {coverage} ± 0.06 over {} cells) \
		 max|grad|={grad:.3} (declared L_t={l_t:.4}) t∈[{t_min:.3},{t_max:.3}] (want 0 exactly, peak >0.5, ≤1)",
		n * n * n
	);
}

#[test]
fn noise_displaced_box_is_closed_and_bit_deterministic() {
	// (b) Deterministic value-noise: two independent builds of the same seeded
	// field must mesh IDENTICALLY (vertex count equal, volume bit-equal) —
	// meshed with Manifold DC, whose run-to-run determinism is pinned by
	// `mdc_determinism.rs`, so any difference here is texture nondeterminism.
	let tex = Texture::Noise { cell: 4.0, seed: 42 };
	let l_t = tex.lipschitz(); // derived: √3/cell ≈ 0.4330
	let mk = || displaced(Node::primitive(Cuboid::new(Vec3::ZERO, Vec3::splat(10.0))), 0.8, tex);
	let region = mk().bounds().pad(0.6);
	let m1 = manifold_dual_contour(&mk(), region, Resolution::VoxelSize(0.4));
	let m2 = manifold_dual_contour(&mk(), region, Resolution::VoxelSize(0.4));
	let (v1, v2) = (m1.signed_volume(), m2.signed_volume());
	let rep = check_mesh(&m1);

	// Outward-only displacement: volume must sit between the bare 20³ box and
	// the box grown by the full amplitude over its whole surface.
	let (box_vol, box_area) = (8000.0_f64, 2400.0_f64);
	let grad = probe_max_grad(|p| tex.value(p), Vec3::splat(-11.0), Vec3::splat(11.0), 30, 0.02);
	let (t_min, t_max) = probe_range(&tex, Vec3::splat(-11.0), Vec3::splat(11.0), 30);

	// Measured (pinned 2026-07-29): both builds 16832 vertices, volume
	// 9025.132316… bit-equal (window 7840..11397); max|∇| 0.334 vs declared
	// 0.433 (the √3 isotropy factor is conservative — per-axis maxima rarely
	// align); t ∈ [0.028, 0.968].
	assert!(
		m1.is_watertight()
			&& rep.boundary_edges == 0
			&& m1.positions.len() == m2.positions.len()
			&& v1.to_bits() == v2.to_bits()
			&& v1 > 0.98 * box_vol
			&& v1 < 1.02 * (box_vol + 0.8 * box_area)
			&& grad <= 1.05 * l_t
			&& t_min >= 0.0
			&& t_max <= 1.0,
		"noise-displaced box: watertight={} boundary_edges={} verts {} vs {} (must be equal) \
		 volume {v1:.6} vs {v2:.6} (must be bit-equal: {} == {}) window {:.0}..{:.0} \
		 noise max|grad|={grad:.4} (declared L_t={l_t:.4}) t∈[{t_min:.3},{t_max:.3}]",
		m1.is_watertight(),
		rep.boundary_edges,
		m1.positions.len(),
		m2.positions.len(),
		v1.to_bits(),
		v2.to_bits(),
		0.98 * box_vol,
		1.02 * (box_vol + 0.8 * box_area)
	);
}

#[test]
fn text_lm10_matches_advance_meshes_closed_and_engraves_a_plate() {
	// (c) "LM-10" at 12 mm capitals, 0.8 mm stroke radius. Font metrics say
	// total advance = 107 grid units · 12/21 ≈ 61.14 mm and ink span
	// [4, 104] units ≈ 57.14 mm, so the meshed bbox width (ink + 2r ≈ 58.7)
	// must land within 10% of the advance.
	let (h, r) = (12.0_f32, 0.8_f32);
	let label = || text_field("LM-10", h, r);
	let adv = text_advance("LM-10", h);
	let mesh = surface_nets(&label(), label().bounds().pad(0.5), Resolution::VoxelSize(0.25));
	let vol = mesh.signed_volume();
	let bb = mesh.aabb();
	let (width, tall) = (bb.max.x - bb.min.x, bb.max.y - bb.min.y);

	// Engrave: plate top face on z = 0 (the text plane), so the difference
	// mills a half-round groove; the plate must LOSE a real groove's volume.
	let plate = || Node::primitive(Cuboid::from_corners(Vec3::new(-3.0, -3.0, -5.0), Vec3::new(64.2, 15.0, 0.0)));
	let region = plate().bounds().pad(0.6);
	let plate_vol = surface_nets(&plate(), region, Resolution::VoxelSize(0.25)).signed_volume();
	let engraved_vol = surface_nets(&plate().difference(label()), region, Resolution::VoxelSize(0.25)).signed_volume();
	let removed = plate_vol - engraved_vol;

	// Measured (pinned 2026-07-29): text vol 247.3 mm³; bbox width 58.72 vs
	// advance 61.14 (−4.0%, inside ±10%); cap span 13.57 (want h+2r = 13.6);
	// engrave removed 128.05 mm³ from plate 6041.6 → 5913.6 — the analytic
	// half-round groove ½·π·r²·Σ|strokes| = ½·π·0.8²·127.37 = 128.04 mm³, a
	// 0.01% match (stroke-joint overlaps ≈ hemispherical stroke-end gains).
	assert!(
		mesh.is_watertight()
			&& vol > 0.0
			&& (width - adv).abs() / adv <= 0.10
			&& (tall - (h + 2.0 * r)).abs() <= 0.4
			&& removed > 100.0
			&& removed < 160.0,
		"text 'LM-10': watertight={} vol={vol:.1} (>0) bbox width={width:.2} vs advance={adv:.2} \
		 ({:+.1}% — must be within ±10%) cap-height+2r={tall:.2} (want {:.2} ± 0.4) \
		 engrave removed {removed:.1} mm³ from plate {plate_vol:.1} → {engraved_vol:.1} (window 100..160, analytic 128.0)",
		mesh.is_watertight(),
		100.0 * (width - adv) / adv,
		h + 2.0 * r
	);
}
