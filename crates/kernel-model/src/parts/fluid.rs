// Copyright (c) LMCAD. Licensed under the MIT License.

//! **Fluid & pneumatic interfaces**: ISO 228-1 parallel pipe-thread (G/BSPP) port
//! bosses with their cited thread/tap-drill table, push-fit pneumatic port cuts
//! (PC4-M6 / PC4-M10, the bowden/airline standard), and parametric **hose barbs**.
//!
//! Thread honesty: as everywhere in this library the helical thread itself is not
//! modelled — a G port is machined/printed at its **tap-drill bore** and the op
//! echoes the full thread row (major Ø, pitch, TPI) so the caller can dimension a
//! drawing or fuse a cosmetic ridge later. Barb proportions are de-facto catalog
//! conventions (stated constants), not a standard.

use super::perp_basis;
use kernel_brep::holes::{drill, HoleDepth};
use kernel_brep::math::{DAffine3, DMat3, DVec2, DVec3};
use kernel_brep::{cylinder, difference, revolve, Solid};

/// One ISO 228-1 parallel pipe thread (BSPP) row: designation, major Ø, threads
/// per inch, pitch (25.4/TPI), and the standard tapping-drill diameter (all mm).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GThreadSpec {
	/// Designation, e.g. `"G1/4"`.
	pub designation: &'static str,
	/// Major (gauge) diameter d.
	pub major_d: f64,
	/// Threads per inch.
	pub tpi: f64,
	/// Pitch, mm (25.4 / TPI).
	pub pitch: f64,
	/// Standard tapping drill Ø for the internal thread.
	pub tap_drill_d: f64,
}

/// ISO 228-1 pipe threads G1/8 … G1/2 `(major Ø, TPI, pitch, tap drill)`.
/// Source: the ISO 228-1 dimension table (major Ø 9.728 / 13.157 / 16.662 /
/// 20.955 at 28 / 19 / 19 / 14 TPI) and the standard BSPP tapping-drill chart
/// (8.8 / 11.8 / 15.25 / 19.0) as published in every thread reference
/// (e.g. Gewinde-Normen / fasteners.eu ISO 228 pages).
const G_THREADS: [GThreadSpec; 4] = [
	GThreadSpec { designation: "G1/8", major_d: 9.728, tpi: 28.0, pitch: 25.4 / 28.0, tap_drill_d: 8.8 },
	GThreadSpec { designation: "G1/4", major_d: 13.157, tpi: 19.0, pitch: 25.4 / 19.0, tap_drill_d: 11.8 },
	GThreadSpec { designation: "G3/8", major_d: 16.662, tpi: 19.0, pitch: 25.4 / 19.0, tap_drill_d: 15.25 },
	GThreadSpec { designation: "G1/2", major_d: 20.955, tpi: 14.0, pitch: 25.4 / 14.0, tap_drill_d: 19.0 },
];

/// The ISO 228-1 row for `"G1/8"`, `"G1/4"`, `"G3/8"` or `"G1/2"` (case-insensitive),
/// or `None`.
pub fn g_thread_spec(designation: &str) -> Option<GThreadSpec> {
	G_THREADS.iter().find(|s| s.designation.eq_ignore_ascii_case(designation)).copied()
}

/// A **G-series (ISO 228-1 / BSPP) pipe-thread port boss**: a round boss of
/// `length` along +Z from z = 0, outer Ø `major + 2·wall`, bored at the standard
/// **tap-drill Ø** straight through, with a 45° entry chamfer at the mouth
/// (z = `length`) opening to the thread major Ø — the lead-in every port print
/// or machining starts from. Union it onto a tank/manifold wall and tap (or
/// thread-mill) the G thread; the sealing face for the bonded washer/O-ring is
/// the flat mouth annulus left outside the chamfer. One revolve: closed,
/// manifold, genus 1, watertight on both routes. The helix is not modelled
/// (documented above). `None` outside the table, for `wall` < 1, or when
/// `length` cannot contain the chamfer.
pub fn pipe_boss_g(designation: &str, wall: f64, length: f64) -> Option<Solid> {
	let spec = g_thread_spec(designation)?;
	let (rb, rm) = (spec.tap_drill_d * 0.5, spec.major_d * 0.5);
	let ro = rm + wall;
	let ch = rm - rb; // 45° chamfer depth
	if !(wall >= 1.0 && length > ch + spec.pitch && length.is_finite()) {
		return None;
	}
	let profile = vec![
		DVec2::new(rb, 0.0),
		DVec2::new(ro, 0.0),
		DVec2::new(ro, length),
		DVec2::new(rm, length), // mouth annulus (sealing face), then 45° in
		DVec2::new(rb, length - ch),
	];
	Some(revolve(&profile, 48))
}

/// Push-fit pneumatic port rows `(thread m, fine pitch, tap drill Ø, pocket
/// depth)`: PC4-M6 — M6×1.0, drill Ø5.0, fitting thread reach ≈ 5 → pocket 6;
/// PC4-M10 — M10×1.0, drill Ø9.0, reach ≈ 6.5 → pocket 7. Source: de-facto
/// pneumatic-fitting datasheets (the SMC/Festo-pattern PC4 bowden/airline
/// fittings reproduced across the 3D-printer ecosystem); both carry 4 mm OD
/// tube, passed below the pocket at Ø4.2 (+0.2 clearance).
const PC4: [(f64, f64, f64, f64); 2] = [(6.0, 1.0, 5.0, 6.0), (10.0, 1.0, 9.0, 7.0)];

/// Tube pass bore below a PC4 pocket: 4 mm OD PTFE/airline + 0.2 clearance.
const PC4_TUBE_PASS_D: f64 = 4.2;

/// Cut a **PC4-M6 / PC4-M10 push-fit pneumatic port** into a planar face: the
/// fitting's tap-drill pocket (flat-bottomed Ø5.0 × 6 for M6×1, Ø9.0 × 7 for
/// M10×1 — tap the fine thread in it) and the Ø4.2 tube-pass bore continuing
/// through the remaining `through − pocket` of material, so the 4 mm tube seats
/// straight through. `at` on the face, `axis` the **outward face normal** (the
/// standard-feature-cut convention, as for the nut trap and servo pocket),
/// `through` the total material depth. Adds one tunnel (genus +1). Thread helix
/// not modelled (tap-drill convention, documented at the module). `None` for
/// `m` outside {6, 10}, a degenerate axis, or `through` not past the pocket.
pub fn pc4_port_cut(solid: &Solid, at: DVec3, axis: DVec3, m: f64, through: f64) -> Option<Solid> {
	let &(_, _, tap_d, pocket) = PC4.iter().find(|r| (r.0 - m).abs() < 1e-9)?;
	let axis = axis.try_normalize()?;
	if !(through > pocket + 0.5 && through.is_finite()) {
		return None;
	}
	// Tube-pass bore through everything first, then the flat-bottomed thread
	// pocket sunk into the face (plain cylinder, 1 mm proud — no drill point).
	let cut = drill(solid, at, -axis, PC4_TUBE_PASS_D, HoleDepth::Through(through), None).ok()?;
	let (e1, e2) = perp_basis(axis);
	let frame = DMat3::from_cols(e1, e2, axis);
	let pocket_cutter = cylinder(DVec3::ZERO, DVec3::Z, tap_d * 0.5, pocket + 1.0, 48)
		.transformed(DAffine3::from_mat3_translation(frame, at - axis * pocket));
	Some(difference(&cut, &pocket_cutter))
}

/// De-facto hose-barb proportions, as fractions of the hose **inner** diameter
/// (mid-range of the common brass barb catalogs; a barb is a convention, not a
/// standard): crest Ø 1.18·ID (the bite), valley Ø 1.00·ID (the hose relaxes to
/// nominal between teeth), bore Ø 0.60·ID, tooth pitch 0.55·ID (75 % gentle
/// ramp toward the tip + square retention shoulder + 25 % valley flat), tip
/// lead-in Ø 0.85·ID, and a 0.50·ID plain stem at the base.
const BARB_CREST: f64 = 1.18;
const BARB_BORE: f64 = 0.60;
const BARB_PITCH: f64 = 0.55;
const BARB_TIP: f64 = 0.85;
const BARB_BASE_RUN: f64 = 0.50;

/// A **hose barb** stem for `hose_id` (the hose's inner Ø) with `barbs` sawtooth
/// teeth: base at z = 0 (union it onto your boss, fitting body or tank wall),
/// tip up, bore Ø `0.6·hose_id` straight through. Teeth follow the documented
/// de-facto proportions ([`BARB_CREST`]): each ramps gently from the valley
/// (hose ID) up to the 118 % crest *toward the base* and ends in a square
/// retention shoulder, so the hose pushes on over the ramps and the shoulders
/// bite on pull-off; the first tooth doubles as the 85 % tip lead-in. One
/// revolve: closed, manifold, genus 1, watertight on both routes. `None` for a
/// degenerate `hose_id` or zero teeth.
pub fn hose_barb(hose_id: f64, barbs: usize) -> Option<Solid> {
	if !(hose_id > 0.0 && hose_id.is_finite() && barbs >= 1) {
		return None;
	}
	let (rv, rc, rb) = (hose_id * 0.5, BARB_CREST * hose_id * 0.5, BARB_BORE * hose_id * 0.5);
	let p = BARB_PITCH * hose_id;
	let len = barbs as f64 * p + BARB_BASE_RUN * hose_id;
	// Outer wall from the tip down (profile assembled CCW: bore up, tip face,
	// outer wall descending): tooth k spans z ∈ [len − (k+1)p, len − kp] — ramp
	// from the tip-side start radius down at the crest, square shoulder, flat.
	let mut profile = vec![DVec2::new(rb, 0.0), DVec2::new(rv, 0.0)];
	let mut outer = Vec::new();
	for k in 0..barbs {
		let z_hi = len - k as f64 * p;
		if k == 0 {
			outer.push(DVec2::new(BARB_TIP * hose_id * 0.5, z_hi)); // tip lead-in
		} // (for k ≥ 1 the ramp starts at the previous valley point, already pushed)
		outer.push(DVec2::new(rc, z_hi - 0.75 * p)); // crest at the ramp foot
		outer.push(DVec2::new(rv, z_hi - 0.75 * p)); // square retention shoulder
		if k + 1 < barbs {
			outer.push(DVec2::new(rv, z_hi - p)); // valley flat to the next tooth
		} // (the last tooth's flat merges into the plain base run)
	}
	// Ascend the outer wall (reverse the descending list), then close over the
	// tip face to the bore.
	profile.extend(outer.into_iter().rev());
	profile.push(DVec2::new(rb, len));
	Some(revolve(&profile, 48))
}

#[cfg(test)]
mod tests {
	use super::*;
	use kernel_brep::{cuboid, tessellate_adaptive_tol, tessellate_default, validate, volume, VertexId};
	use std::f64::consts::PI;

	/// `(closed && manifold && genus == want && watertight on both routes, validity)`.
	fn check(s: &Solid, want_genus: i64) -> (bool, String) {
		let v = validate(s);
		let ok = v.closed
			&& v.manifold && v.genus == want_genus
			&& tessellate_default(s).is_watertight()
			&& tessellate_adaptive_tol(s, 0.01).is_watertight();
		(ok, format!("{v:?} wt={} adaptive_wt={}", tessellate_default(s).is_watertight(), tessellate_adaptive_tol(s, 0.01).is_watertight()))
	}

	/// ∫R² over a linear ramp from r0 to r1 across height h (frustum integral).
	fn frustum(r0: f64, r1: f64, h: f64) -> f64 {
		h / 3.0 * (r0 * r0 + r0 * r1 + r1 * r1)
	}

	/// 48-gon area factor: a revolved cross-section at radius r has area `c48()·r²`,
	/// and a frustum panel between aligned 48-gon rings is exactly planar, so
	/// piecewise `c48()·∫R(z)² dz` is the exact solid volume.
	fn c48() -> f64 {
		24.0 * (2.0 * PI / 48.0).sin()
	}

	#[test]
	fn g_table_carries_iso228_and_bosses_bore_at_tap_drill() {
		// The four ISO 228-1 rows (major Ø, TPI, tap drill) and pitch = 25.4/TPI;
		// bosses for G1/8 (wall 2 × 10) and G1/2 (wall 3 × 15): genus-1 watertight×2
		// revolves spanning tap/2 … major/2 + wall with the 45° mouth chamfer, and
		// the exact 48-gon closed-form volume (tube − chamfer ring) to 1e-6.
		let rows: Vec<_> = ["G1/8", "G1/4", "G3/8", "G1/2", "G3/4"]
			.iter()
			.map(|d| g_thread_spec(d).map(|s| (s.major_d, s.tpi, s.tap_drill_d)))
			.collect();
		assert_eq!(
			rows,
			vec![
				Some((9.728, 28.0, 8.8)),
				Some((13.157, 19.0, 11.8)),
				Some((16.662, 19.0, 15.25)),
				Some((20.955, 14.0, 19.0)),
				None
			],
			"ISO 228-1 G-thread rows (G3/4 not stocked)"
		);
		for (des, wall, len) in [("G1/8", 2.0, 10.0), ("G1/2", 3.0, 15.0)] {
			let spec = g_thread_spec(des).expect("row");
			let b = pipe_boss_g(des, wall, len).expect("boss");
			let (ok, diag) = check(&b, 1);
			let (rb, rm) = (spec.tap_drill_d * 0.5, spec.major_d * 0.5);
			let ro = rm + wall;
			let ch = rm - rb;
			let expected = c48() * (ro * ro * len - (rb * rb * (len - ch) + frustum(rb, rm, ch)));
			let vol = volume(&b).abs();
			let rmax = (0..b.vertex_count() as u32)
				.map(|i| {
					let p = b.position(VertexId(i));
					(p.x * p.x + p.y * p.y).sqrt()
				})
				.fold(0.0_f64, f64::max);
			assert!(
				ok && (rmax - ro).abs() < 1e-9 && (vol - expected).abs() / expected < 1e-6,
				"{des} boss: want watertight×2 genus-1, OD {}, exactly {expected:.3}mm³; got {diag} rmax={rmax} vol={vol:.3}",
				2.0 * ro
			);
		}
		assert!(pipe_boss_g("G1/4", 0.5, 10.0).is_none(), "sub-1 mm walls are refused");
	}

	#[test]
	fn pc4_ports_pocket_then_pass_the_tube() {
		// A 10-thick manifold plate takes one M6 and one M10 port: each adds one
		// tunnel (genus 2 total), pocket floors land at exactly z = 10 − depth with
		// the Ø-tap wall above and the Ø4.2 pass below; volume = plate − pockets −
		// passes within 1%. Too-thin material and odd threads are refused.
		let plate = cuboid(DVec3::new(-30.0, -15.0, 0.0), DVec3::new(30.0, 15.0, 10.0));
		let m6 = pc4_port_cut(&plate, DVec3::new(-15.0, 0.0, 10.0), DVec3::Z, 6.0, 10.0).expect("M6 port");
		let both = pc4_port_cut(&m6, DVec3::new(15.0, 0.0, 10.0), DVec3::Z, 10.0, 10.0).expect("M10 port");
		let (ok, diag) = check(&both, 2);
		let floor_m6 = (0..both.vertex_count() as u32)
			.map(|i| both.position(VertexId(i)))
			.filter(|p| {
				let r = ((p.x + 15.0) * (p.x + 15.0) + p.y * p.y).sqrt();
				(r - 2.5).abs() < 1e-9
			})
			.map(|p| p.z)
			.fold(f64::INFINITY, f64::min);
		let expected = 60.0 * 30.0 * 10.0
			- PI * 2.5 * 2.5 * 6.0 - PI * 4.5 * 4.5 * 7.0 // pockets
			- PI * 2.1 * 2.1 * (4.0 + 3.0); // tube passes below the pockets
		let vol = volume(&both).abs();
		assert!(
			ok && (floor_m6 - 4.0).abs() < 1e-9
				&& (vol - expected).abs() / expected < 0.01
				&& pc4_port_cut(&plate, DVec3::new(0.0, 0.0, 10.0), DVec3::Z, 6.0, 6.0).is_none()
				&& pc4_port_cut(&plate, DVec3::new(0.0, 0.0, 10.0), DVec3::Z, 8.0, 10.0).is_none(),
			"PC4 ports: want watertight×2 genus-2 with the M6 pocket floor at z=4, ~{expected:.0}mm³ (and refusals for through ≤ pocket, M8); got {diag} floor={floor_m6} vol={vol:.0}"
		);
	}

	#[test]
	fn hose_barbs_bite_at_118_percent_and_stay_watertight() {
		// Ø6 × 3 teeth and Ø8 × 4 teeth: genus-1 watertight×2 revolves reaching
		// exactly the 1.18 crest radius on every tooth, bore exactly 0.6·ID, and the
		// exact piecewise-frustum 48-gon volume to 1e-6.
		for (id, n) in [(6.0, 3_usize), (8.0, 4)] {
			let b = hose_barb(id, n).expect("barb");
			let (ok, diag) = check(&b, 1);
			let (rv, rc, rb) = (id * 0.5, BARB_CREST * id * 0.5, BARB_BORE * id * 0.5);
			let p = BARB_PITCH * id;
			let len = n as f64 * p + BARB_BASE_RUN * id;
			// Outer ∫R²dz: base run + per tooth (ramp frustum + its 0.25p valley
			// flat — the last tooth's flat is the merged edge above the base run),
			// minus the bore.
			let mut int = rv * rv * (BARB_BASE_RUN * id);
			for k in 0..n {
				let start_r = if k == 0 { BARB_TIP * id * 0.5 } else { rv };
				int += frustum(rc, start_r, 0.75 * p) + rv * rv * 0.25 * p;
			}
			let expected = c48() * (int - rb * rb * len);
			let vol = volume(&b).abs();
			let crest_verts = (0..b.vertex_count() as u32)
				.map(|i| b.position(VertexId(i)))
				.filter(|q| ((q.x * q.x + q.y * q.y).sqrt() - rc).abs() < 1e-9)
				.count();
			assert!(
				ok && crest_verts >= 48 * n && (vol - expected).abs() / expected < 1e-6,
				"barb Ø{id}×{n}: want watertight×2 genus-1 with {n} crests at Ø{:.2}, exactly {expected:.3}mm³; got {diag} crest_verts={crest_verts} vol={vol:.3}",
				2.0 * rc
			);
		}
		assert!(hose_barb(6.0, 0).is_none() && hose_barb(0.0, 3).is_none(), "degenerate barbs are refused");
	}
}
