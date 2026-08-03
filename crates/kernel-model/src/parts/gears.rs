// Copyright (c) LMCAD. Licensed under the MIT License.

//! Involute gearing: spur gears, the **gear rack** (the basic rack itself — exact, since the
//! rack IS the involute's straight-flank limit) and **internal ring gears**. Tooth flanks of
//! the round gears are true involutes of the base circle (sampled into the profile polygon);
//! proportions follow the ISO 53 basic rack: addendum `1·m`, dedendum `1.25·m`, standard
//! pressure angle 20° (parametric).

use super::shafts::KeySize;
use kernel_brep::math::{DVec2, DVec3};
use kernel_brep::{extrude_with_holes, Solid};
use std::f64::consts::{PI, TAU};

/// One point of an involute unrolled from a base circle of radius `rb`, at roll parameter `t`
/// (radians of unwound arc). Its distance from the centre is `rb·√(1+t²)` and its polar angle
/// is the involute function `inv(t) = t − atan(t)`.
fn involute(rb: f64, t: f64) -> DVec2 {
	DVec2::new(rb * (t.cos() + t * t.sin()), rb * (t.sin() - t * t.cos()))
}

/// Rotate a 2D point by angle `a` about the origin.
fn rot(p: DVec2, a: f64) -> DVec2 {
	DVec2::new(p.x * a.cos() - p.y * a.sin(), p.x * a.sin() + p.y * a.cos())
}

/// A parametric **involute spur gear**: `module` × `teeth` with true involute flanks at the
/// given pressure angle, extruded to `face_width`, bored at `bore_d` diameter, optionally with
/// a DIN 6885 hub keyway (width `b`, ceiling at `bore_d/2 + t2` — pass
/// [`super::din6885_key_size`]`(bore_d)` for the standard size). Proportions per the ISO 53
/// basic rack: tip radius `m(z/2 + 1)`, root radius `m(z/2 − 1.25)`.
///
/// Construction: one profile polygon per gear — root land arcs, involute flanks (8 samples
/// each, machine-exact endpoints), tip land arcs — extruded solid, then the bore cut by one
/// exact boolean difference (genus 1). A plain bore is cut with the analytic `cylinder`
/// primitive (the same 48-gon wall as the library's `circle48`, but carrying the exact
/// cylinder surface tag, so `exact_volume` is π-exact on the bore and STEP export writes a
/// true cylinder); a keyed bore is a keyway-notched polygonal prism cut. The boolean route —
/// not an `extrude_with_holes` hole loop — keeps the caps free of inner loops, so the
/// adaptive tessellation is watertight and STL export takes the exact route, not the voxel
/// heal (FRICTION.md #6).
///
/// Honest approximations (documented, not silent):
/// - the **root fillet is optional** — off by default (`spur_gear` leaves the historical sharp
///   radial root), on via [`spur_gear_filleted`] / [`involute_ring_outline_shifted_filleted`],
///   which round the foot with a tangent circular arc (the circular approximation of the hob
///   trochoid — the single highest-value tooth-strength feature for printed gears);
/// - **undercut is not modelled**: below ~17 teeth (at 20°) a generated gear would undercut;
///   this profile keeps the pure involute, so very small tooth counts are geometrically valid
///   solids but not kinematically exact gears;
/// - flanks below the base circle (when the root lies inside it) are radial lines, the standard
///   drafting simplification.
///
/// The caller must keep `bore_d/2 + t2 < m(z/2 − 1.25)` (keyway inside the dedendum) for a
/// sane part; the function builds exactly what is asked.
/// The **cycloidal-drive disc profile**: the classic epitrochoid of a disc
/// with `lobes` lobes meshing `lobes + 1` ring pins of radius `pin_r` on a
/// circle of radius `ring_r`, at eccentricity `ecc` — the reduction is exactly
/// `lobes : 1` with the ring fixed and the disc creeping `−θ/lobes` per cam
/// angle θ. `pin_r` here is the EFFECTIVE pin radius (inflate it by your
/// meshing clearance to shrink the disc). Returns `pts_per_lobe · lobes`
/// exact points on the curve, in order. Feasibility (cusp `ecc < ring_r/N`,
/// offset simplicity) is the caller's to assert — this is the raw curve.
pub fn cycloid_disc_profile(lobes: usize, ring_r: f64, pin_r: f64, ecc: f64, pts_per_lobe: usize) -> Vec<DVec2> {
	let n = (lobes + 1) as f64;
	let total = lobes * pts_per_lobe;
	let mut out = Vec::with_capacity(total);
	for k in 0..total {
		let t = TAU * k as f64 / total as f64;
		let a = (1.0 - n) * t;
		let psi = f64::atan2(a.sin(), ring_r / (ecc * n) - a.cos());
		let x = ring_r * t.cos() - pin_r * (t + psi).cos() - ecc * (n * t).cos();
		let y = -ring_r * t.sin() + pin_r * (t + psi).sin() + ecc * (n * t).sin();
		out.push(DVec2::new(x, y));
	}
	out
}

pub fn spur_gear(module: f64, teeth: usize, face_width: f64, bore_d: f64, pressure_angle_deg: f64, keyway: Option<KeySize>) -> Solid {
	spur_gear_filleted(module, teeth, face_width, bore_d, pressure_angle_deg, keyway, 0.0)
}

/// [`spur_gear`] with an optional **circular root fillet** of radius `root_fillet_coeff · module`
/// at the tooth feet (`0.0` = the sharp-root [`spur_gear`], byte-identical). The fillet is the
/// highest-value strength feature for a printed gear: it removes the sharp root corner where the
/// Lewis critical section sits and where FDM parts crack, recovering most of the bending capacity
/// that sharp-root generators silently lose versus handbook (hobbed-tooth) ratings. Typical
/// values 0.2–0.4; the generator clamps per tooth and keeps a sharp root wherever the fillet
/// would not fit (documented in [`involute_outline_df`]).
pub fn spur_gear_filleted(module: f64, teeth: usize, face_width: f64, bore_d: f64, pressure_angle_deg: f64, keyway: Option<KeySize>, root_fillet_coeff: f64) -> Solid {
	let rp = module * teeth as f64 / 2.0;
	let rf = (root_fillet_coeff * module).max(0.0);
	let poly = involute_outline_df(module, teeth, pressure_angle_deg.to_radians(), rp + module, rp - 1.25 * module, 0.0, rf);
	match keyway {
		None => super::extrude_bored(&poly, face_width, &[(DVec2::ZERO, bore_d * 0.5, 48)], &[]),
		Some(_) => super::extrude_bored(&poly, face_width, &[], &[bore_with_keyway(bore_d * 0.5, keyway)]),
	}
}

/// The closed involute-toothed outline shared by external and internal gears: `z` teeth of
/// true-involute flanks (base circle from the pitch radius `m·z/2` and `alpha`), with tip
/// land arcs at radius `ra` and root land arcs at `rr`, teeth centred on polar angles
/// `k·2π/z` (tooth 0 on +X). The tooth half-thickness at the pitch circle is exactly a
/// quarter pitch (`π·m/2` thickness — the zero-backlash nominal), which makes the SAME
/// generator serve the internal gear: a ring's tooth *space* is an external-style "air
/// tooth" with `ra` at the ring root `rp + 1.25m` and `rr` at the ring tip `rp − m`.
/// The closed involute gear outline as a bare polygon — the building block for
/// COMPOUND gear parts (stepped planets, ring gears merged into housings/output
/// hubs) and for exact 2D mesh simulation. `external` selects standard external
/// proportions (tip `rp+m`, root `rp−1.25m`) or the INTERNAL-gear cavity outline
/// (the "hole": tip `rp+1.25m`, root `rp−m`). `half_pitch_shift` rotates the
/// pattern half a pitch so a tooth SPACE (not a tooth) sits on +X — the phase a
/// mating external gear on the +X line of centers needs. `None` when the
/// geometry pinches (see `internal_gear`).
/// `thin_mm`: circumferential backlash allowance per flank at the pitch line —
/// EXTERNAL teeth get thinner, an INTERNAL cavity's tooth spaces get wider
/// (both remove metal). Zero reproduces the exact half-pitch tooth of
/// `spur_gear`/`internal_gear` (which mesh at zero backlash — printable
/// gears need ~0.05).
///
/// `shift_coeff` (x): the **ISO 53 profile-shift coefficient**. A positive x
/// displaces the reference rack radially OUTWARD (away from the gear axis) by
/// `x·m` for BOTH member types, so:
/// - both radial extents grow by `x·m` — EXTERNAL tip `ra = rp + m(1+x)`, root
///   `rr = rp − m(1.25−x)`; INTERNAL cavity outer (ring root) `rp + m(1.25+x)`,
///   inner (ring tip) `rp − m(1−x)`;
/// - each flank's pitch-line half-thickness grows by `x·m·tan α` (tooth thickness
///   `+2·x·m·tan α`), realised as an angular widening `x·m·tan α / rp` of `half`.
///
/// **Internal-gear convention (the mirror, derived so equal shifts preserve the
/// mesh):** an internal gear's tooth is the rack's SPACE, so on the ring the same
/// `+x·m·tan α` widening WIDENS the tooth space (THINS the ring tooth). That is
/// exactly the sign that keeps an EXTERNAL-positive-shift pinion of the SAME x
/// meshing at the UNCHANGED operating pitch / centre distance: for an internal
/// pair `inv(αw) = inv(α) + 2·tan α·(x_ring − x_pinion)/(z_ring − z_pinion)`, so
/// equal shifts (`x_ring = x_pinion`) give `αw = α` — standard centre distance,
/// constant `2m` working depth. (Verified as ground truth by the PLAN-26
/// simulator's S1 interference sweep.) `shift_coeff = 0` reproduces the
/// unshifted outline byte-for-byte (the added terms are exactly `0.0`), so every
/// existing caller — via [`involute_ring_outline_thinned`] — is unchanged.
pub fn involute_ring_outline_shifted(module: f64, teeth: usize, pressure_angle_deg: f64, external: bool, half_pitch_shift: bool, thin_mm: f64, shift_coeff: f64) -> Option<Vec<DVec2>> {
	involute_ring_outline_shifted_filleted(module, teeth, pressure_angle_deg, external, half_pitch_shift, thin_mm, shift_coeff, 0.0)
}

/// [`involute_ring_outline_shifted`] plus an optional **circular root fillet** at the tooth
/// feet, radius `root_fillet_coeff · module` (millimetres). The fillet is the single
/// highest-value tooth-strength feature for printed gears: it replaces the sharp radial
/// root corner — where the real Lewis critical section sits and where a printed part
/// cracks — with a tangent arc, lifting root-bending capacity materially (a well-filleted
/// root recovers most of the gap between the sharp-corner form factor and the hobbed-tooth
/// handbook value). Applied to EXTERNAL teeth only (`external == true`) — the bending-critical
/// members (sun/planet/pinion); an internal ring's cavity is not filleted (its own root is the
/// stronger outer rim, and rounding the "air tooth" would round the ring tips, not roots).
/// `root_fillet_coeff == 0.0` is byte-identical to [`involute_ring_outline_shifted`]. Typical
/// values 0.2–0.4·m; the generator clamps per tooth and keeps the sharp root wherever the
/// fillet would overrun the root land or the base circle (documented, never a silent
/// self-intersection). See [`involute_outline_df`] for the exact construction.
#[allow(clippy::too_many_arguments)] // each arg is an independent gear-profile dimension
pub fn involute_ring_outline_shifted_filleted(module: f64, teeth: usize, pressure_angle_deg: f64, external: bool, half_pitch_shift: bool, thin_mm: f64, shift_coeff: f64, root_fillet_coeff: f64) -> Option<Vec<DVec2>> {
	let (m, z) = (module, teeth);
	let rp = m * z as f64 / 2.0;
	let alpha = pressure_angle_deg.to_radians();
	if z < 8 || !(m > 0.0 && alpha > 0.0 && alpha < 0.6) {
		return None;
	}
	// Profile shift x·m: reference rack moved radially outward by x·m — both radial
	// extents grow by x·m for either member type (adds exactly 0.0 when x == 0, so
	// the unshifted output is byte-identical).
	let sh = shift_coeff * m;
	let (ra, rr) = if external { (rp + m + sh, rp - 1.25 * m + sh) } else { (rp + 1.25 * m + sh, rp - m + sh) };
	// per-flank angular widening at the pitch line from the shift (x·m·tan α / rp)
	let shift_half = shift_coeff * m * alpha.tan() / rp;
	if !external {
		let rb = rp * alpha.cos();
		let t_root = ((ra / rb).powi(2) - 1.0).sqrt();
		let half = PI / (2.0 * z as f64) + (alpha.tan() - alpha) + shift_half;
		if half <= t_root - t_root.atan() {
			return None;
		}
	}
	let rp_ = m * z as f64 / 2.0;
	let dhalf = (if external { -thin_mm / rp_ } else { thin_mm / rp_ }) + shift_half;
	// Fillet external teeth only; an internal cavity keeps its sharp (stronger-rim) root.
	let rf = if external { (root_fillet_coeff * m).max(0.0) } else { 0.0 };
	let mut pts = involute_outline_df(m, z, alpha, ra, rr, dhalf, rf);
	if half_pitch_shift {
		let s = PI / z as f64;
		let (c, sn) = (s.cos(), s.sin());
		for p in &mut pts {
			*p = DVec2::new(p.x * c - p.y * sn, p.x * sn + p.y * c);
		}
	}
	Some(pts)
}

/// Zero-profile-shift wrapper over [`involute_ring_outline_shifted`]: the
/// backlash-thinned outline every existing caller builds from. Byte-identical to
/// the pre-profile-shift generator (`shift_coeff = 0.0`).
pub fn involute_ring_outline_thinned(module: f64, teeth: usize, pressure_angle_deg: f64, external: bool, half_pitch_shift: bool, thin_mm: f64) -> Option<Vec<DVec2>> {
	involute_ring_outline_shifted(module, teeth, pressure_angle_deg, external, half_pitch_shift, thin_mm, 0.0)
}

/// Backward-compatible zero-backlash variant.
pub fn involute_ring_outline(module: f64, teeth: usize, pressure_angle_deg: f64, external: bool, half_pitch_shift: bool) -> Option<Vec<DVec2>> {
	involute_ring_outline_thinned(module, teeth, pressure_angle_deg, external, half_pitch_shift, 0.0)
}

fn involute_outline(m: f64, z: usize, alpha: f64, ra: f64, rr: f64) -> Vec<DVec2> {
	involute_outline_df(m, z, alpha, ra, rr, 0.0, 0.0)
}

/// A single **circular root-fillet arc**, tangent to the root circle (radius `rr`) and to the
/// radial tooth foot at polar angle `foot`, rounding the concave root corner with radius `rf`.
/// The fillet centre sits in the tooth space at radius `rr + rf` and polar angle `phi_c`
/// (= `foot ± δ`, `δ = asin(rf/(rr+rf))`); the arc runs between the root-circle tangent point
/// `(rr, phi_c)` and the flank-foot tangent point `(rft, foot)` with `rft = √(rr²+2·rr·rf)`.
/// Emitted in increasing-polar-angle order. `flank_first` puts the flank-foot end first (a
/// right-flank foot, gap ahead); otherwise the root end is first (a left-flank foot). When
/// `skip_first` the leading endpoint is dropped so it does not duplicate the caller's previous
/// point (the shared root-land vertex). Points are the exact circle — a machine-round fillet,
/// the widely-used circular approximation of the hob trochoid.
#[allow(clippy::too_many_arguments)] // root circle + fillet geometry + emission flags + sink
fn root_fillet_arc(rr: f64, rf: f64, foot: f64, phi_c: f64, rft: f64, flank_first: bool, skip_first: bool, out: &mut Vec<DVec2>) {
	let center = DVec2::new((rr + rf) * phi_c.cos(), (rr + rf) * phi_c.sin());
	let p_root = DVec2::new(rr * phi_c.cos(), rr * phi_c.sin());
	let p_flank = DVec2::new(rft * foot.cos(), rft * foot.sin());
	let (v0, v1) = (p_flank - center, p_root - center); // both length rf
	let (a_flank, a_root) = (v0.y.atan2(v0.x), v1.y.atan2(v1.x));
	let mut d = a_root - a_flank;
	while d > PI {
		d -= TAU;
	}
	while d < -PI {
		d += TAU;
	}
	const N: usize = 4;
	for j in 0..=N {
		if skip_first && j == 0 {
			continue;
		}
		// flank_first: sweep a_flank → a_root; else root → flank (so the whole arc still
		// ascends in polar angle either way — the left/right feet mirror).
		let f = if flank_first { j as f64 } else { (N - j) as f64 } / N as f64;
		let a = a_flank + d * f;
		out.push(center + DVec2::new(rf * a.cos(), rf * a.sin()));
	}
}

/// The involute tooth outline with an optional **circular root fillet** of radius `rf`
/// (millimetres) at each flank foot. `rf == 0.0` reproduces the sharp-root outline
/// byte-for-byte (the fillet branch is never entered), so every existing caller is unchanged.
/// The fillet is only applied where a radial tooth foot exists — i.e. the root lies at or
/// inside the base circle (`rr < rb`, the common low-tooth-count regime and exactly the case
/// the drive audit flagged); above the base circle, or when `rf` would overrun the root land
/// or the base circle, the sharp root is kept for that tooth (documented fallback, never a
/// silent self-intersection).
fn involute_outline_df(m: f64, z: usize, alpha: f64, ra: f64, rr: f64, dhalf: f64, rf: f64) -> Vec<DVec2> {
	let rp = m * z as f64 / 2.0; // pitch radius
	let rb = rp * alpha.cos(); // base radius
	// Roll parameter where the involute reaches the tip, and where it starts: at the base
	// circle (t = 0) when the root is inside it, else already out at the root radius.
	let t_tip = ((ra / rb).powi(2) - 1.0).sqrt();
	let t_start = if rr > rb { ((rr / rb).powi(2) - 1.0).sqrt() } else { 0.0 };
	let inv = |t: f64| t - t.atan(); // involute polar-angle function
	let (theta_tip, inv0) = (inv(t_tip), inv(t_start));
	// Angular half-width of a tooth at the base circle: half the pitch-circle tooth thickness
	// (π/2z) plus the involute spread inv(α) rolled back from pitch to base.
	let half = PI / (2.0 * z as f64) + (alpha.tan() - alpha) + dhalf;
	let pitch = 2.0 * PI / z as f64;
	// Root fillet is only geometrically well-posed when a radial foot exists (root inside the
	// base circle). δ is the polar half-span the fillet consumes on the root circle; rft the
	// radius at which it meets the radial foot.
	let fillet = rf > 0.0 && rr < rb;
	let (delta, rft) = if fillet { ((rf / (rr + rf)).asin(), (rr * rr + 2.0 * rr * rf).sqrt()) } else { (0.0, rr) };

	let mut poly: Vec<DVec2> = Vec::new();
	for k in 0..z {
		let c = k as f64 * pitch;
		let (a_l, a_r) = (c - half, c + half);
		// Root land: an arc at rr spanning the gap from the previous tooth's right flank
		// (polar a_r − pitch − inv0) to this tooth's left flank start (polar a_l + inv0).
		// When the root lies outside the base circle (inv0 > 0) the arc's endpoints coincide
		// with the flanks' start points and the profile sanitiser drops the duplicates;
		// otherwise the flanks start at the base circle and the joins are radial steps.
		let (g0, g1) = (a_l - (pitch - 2.0 * half) - inv0, a_l + inv0);
		// Fillet only if it fits: it must leave positive root land between the two feet and
		// meet the radial foot below the base circle. Otherwise keep the sharp root here.
		if fillet && 2.0 * delta < (g1 - g0) - 1e-9 && rft < rb {
			// g0 end (previous tooth's right-flank foot, gap ahead): descend the residual radial
			// foot rb → rft (collinear with the fillet tangent, just sampled so it is not one
			// coarse edge), then the fillet arc flank → root.
			for j in 1..3 {
				let r = rb + (rft - rb) * j as f64 / 3.0;
				poly.push(DVec2::new(r * g0.cos(), r * g0.sin()));
			}
			root_fillet_arc(rr, rf, g0, g0 + delta, rft, true, false, &mut poly);
			// Root land, trimmed to the two fillet tangent points (skip the shared first pt).
			for j in 1..=4 {
				let a = (g0 + delta) + ((g1 - delta) - (g0 + delta)) * j as f64 / 4.0;
				poly.push(DVec2::new(rr * a.cos(), rr * a.sin()));
			}
			// g1 end (this tooth's left-flank foot, gap behind): fillet root → flank (skip shared
			// pt), then ascend the residual radial foot rft → rb up to the involute start.
			root_fillet_arc(rr, rf, g1, g1 - delta, rft, false, true, &mut poly);
			for j in 1..3 {
				let r = rft + (rb - rft) * j as f64 / 3.0;
				poly.push(DVec2::new(r * g1.cos(), r * g1.sin()));
			}
		} else {
			for j in 0..=4 {
				let a = g0 + (g1 - g0) * j as f64 / 4.0;
				poly.push(DVec2::new(rr * a.cos(), rr * a.sin()));
			}
		}
		// Left flank: involute from t_start out to the tip.
		for j in 0..=7 {
			let t = t_start + (t_tip - t_start) * j as f64 / 7.0;
			poly.push(rot(involute(rb, t), a_l));
		}
		// Tip land: an arc at ra between the two flank tips.
		for j in 1..=2 {
			let a = (a_l + theta_tip) + ((a_r - theta_tip) - (a_l + theta_tip)) * j as f64 / 3.0;
			poly.push(DVec2::new(ra * a.cos(), ra * a.sin()));
		}
		// Right flank: the mirrored involute, traversed tip → start so the angle ascends.
		for j in (0..=7).rev() {
			let t = t_start + (t_tip - t_start) * j as f64 / 7.0;
			let i = involute(rb, t);
			poly.push(rot(DVec2::new(i.x, -i.y), a_r));
		}
	}
	poly
}

/// An **internal (ring) gear**: `teeth` involute tooth spaces cut into the bore of an
/// annular rim of outer Ø `rim_od`, extruded to `face_width`. Internal-gear proportions per
/// the ISO 53 conventions: tip circle Ø `m(z − 2)` (teeth point inward), root circle Ø
/// `m(z + 2.5)`. The bore outline is the involute "air gear" of the same base circle (see
/// [`involute_outline`]), so the flanks are exact conjugates of a [`spur_gear`] pinion of
/// the same module and pressure angle at centre distance `(z_ring − z_pinion)·m/2` — the
/// tests mesh the pair and assert zero interpenetration. Genus 1. The rim blank is the
/// analytic `cylinder` primitive (exact-cylinder OD surface tags) and the toothed bore is
/// one exact boolean prism cut — not an `extrude_with_holes` hole loop — so the caps carry
/// no inner loops, the adaptive tessellation stays watertight and STL export routes exact
/// instead of voxel-healed (FRICTION.md #6).
///
/// Honest approximations (same family as [`spur_gear`], documented, not silent): no root
/// fillets, radial flank feet below the base circle, and **tip fouling is not checked** —
/// small tooth differences (ring − pinion ≲ 10) physically foul at assembly; the function
/// builds what is asked. `None` when the rim wall would vanish (`rim_od ≤ m(z + 2.5)`),
/// for degenerate dimensions, or `teeth < 8`.
pub fn internal_gear(module: f64, teeth: usize, face_width: f64, rim_od: f64, pressure_angle_deg: f64) -> Option<Solid> {
	let (m, z) = (module, teeth);
	let rp = m * z as f64 / 2.0;
	let (r_root, r_tip) = (rp + 1.25 * m, rp - m);
	let alpha = pressure_angle_deg.to_radians();
	// NaN-safe rejection: conjunctions refuse non-finite input too.
	if z < 8
		|| !(m > 0.0
			&& face_width > 0.0 && face_width.is_finite()
			&& rim_od * 0.5 > r_root
			&& rim_od.is_finite()
			&& alpha > 0.0 && alpha < 0.6)
	{
		return None;
	}
	// The ring's root land must not vanish: the air-tooth's land arc at r_root has angular
	// width 2·(half − inv(t_root)), which pinches to zero near α = 30° (e.g. z 36) — refuse
	// instead of emitting a self-intersecting outline.
	let rb = rp * alpha.cos();
	let t_root = ((r_root / rb).powi(2) - 1.0).sqrt();
	let half = PI / (2.0 * z as f64) + (alpha.tan() - alpha);
	if half <= t_root - t_root.atan() {
		return None;
	}
	let hole = involute_outline(m, z, alpha, r_root, r_tip);
	let blank = kernel_brep::cylinder(DVec3::ZERO, DVec3::Z, rim_od * 0.5, face_width, 48);
	let cutter = kernel_brep::extrude(&hole, face_width + 2.0)
		.transformed(kernel_brep::math::DAffine3::from_translation(DVec3::new(0.0, 0.0, -1.0)));
	Some(kernel_brep::difference(&blank, &cutter))
}

/// A **gear rack** of the ISO 53 / DIN 867 basic-rack tooth form — straight flanks at the
/// pressure angle, which IS the exact involute profile for infinite radius: pitch `π·m`,
/// addendum `1·m` above the pitch line, dedendum `1.25·m` below, tooth thickness at the
/// pitch line exactly half a pitch (zero-backlash nominal). The bar lies along +X spanning
/// `[0, length]`, teeth point +Y, extruded `width` along +Z; the back face is y = 0, the
/// root line y = `1.75·m` (a conventional `1.75·m` body under the root, overall section
/// height `4·m`) and the **pitch line y = `3·m`** — mesh a [`spur_gear`] by placing its
/// axis at pitch-line height plus its pitch radius.
///
/// Only **whole teeth** are cut, the pattern centred along the bar with flat root-level
/// lands at the ends (racks are cropped mid-gap; no partial trapezoids). Honest omissions:
/// the ISO 53 root/tip fillets (`ρ ≈ 0.38·m`) are left sharp, as across this gear family.
/// All faces planar, so the closed-form volume is exact — asserted in the tests. `None`
/// for degenerate dimensions, a pressure angle outside (0°, 32°) (above ~32.1° adjacent
/// root corners merge), or a bar too short for one whole tooth.
pub fn gear_rack(module: f64, length: f64, width: f64, pressure_angle_deg: f64) -> Option<Solid> {
	let m = module;
	let alpha = pressure_angle_deg.to_radians();
	let p = PI * m; // pitch
	let w_tip = 0.25 * p - m * alpha.tan(); // tooth half-thickness at the tip line
	let w_root = 0.25 * p + 1.25 * m * alpha.tan(); // tooth half-thickness at the root line
	let root_gap = p - 2.0 * w_root; // flat between adjacent root corners
	// NaN-safe rejection: conjunctions refuse non-finite or NaN input too.
	if !(m > 0.0 && length.is_finite() && width > 0.0 && width.is_finite() && alpha > 0.0 && w_tip > 0.0 && root_gap > 0.0) {
		return None;
	}
	if length < 2.0 * w_root {
		return None; // not even one whole tooth fits
	}
	let n = ((length - 2.0 * w_root) / p).floor() as usize + 1;
	let (root_y, tip_y) = (1.75 * m, 4.0 * m);
	let mut poly = vec![DVec2::new(0.0, 0.0), DVec2::new(length, 0.0), DVec2::new(length, root_y)];
	// Top edge, traversed right → left so the outline stays counter-clockwise.
	for k in (0..n).rev() {
		let xc = 0.5 * length + (k as f64 - 0.5 * (n as f64 - 1.0)) * p;
		poly.push(DVec2::new(xc + w_root, root_y));
		poly.push(DVec2::new(xc + w_tip, tip_y));
		poly.push(DVec2::new(xc - w_tip, tip_y));
		poly.push(DVec2::new(xc - w_root, root_y));
	}
	poly.push(DVec2::new(0.0, root_y));
	Some(extrude_with_holes(&poly, &[], width))
}

/// The hub bore as one hole loop: a circle of radius `r` (48 segments), optionally notched by a
/// rectangular DIN 6885 hub keyway of width `key.b` reaching out to `r + key.t2` on the +X
/// side. (Plain bored hubs elsewhere in the library use [`super::circle48`] directly.)
fn bore_with_keyway(r: f64, keyway: Option<KeySize>) -> Vec<DVec2> {
	let Some(key) = keyway else {
		return super::circle48(r);
	};
	// The notch replaces the arc where |y| ≤ b/2 on the +X side: walk the circle CCW from the
	// notch's upper corner round to its lower corner, then jump out to the keyway ceiling at
	// x = r + t2.
	let mut hole = Vec::with_capacity(48);
	let hw = key.b * 0.5;
	let a0 = (hw / r).asin();
	let n = 44;
	for i in 0..=n {
		let a = a0 + (2.0 * PI - 2.0 * a0) * i as f64 / n as f64;
		hole.push(DVec2::new(r * a.cos(), r * a.sin()));
	}
	hole.push(DVec2::new(r + key.t2, -hw));
	hole.push(DVec2::new(r + key.t2, hw));
	hole
}

/// One straight-flank (trapezoid) tooth's profile as `(angular_offset_rad,
/// radius_mm)` points, CCW, for a tooth centred at angle 0 — the SINGLE SOURCE
/// of the strain-wave (harmonic) drive's tooth geometry, shared by the printed
/// housing (internal circular spline), the printed flexspline (external), and
/// the kinematic simulator's deformed-tooth model. Sharing one function is the
/// structural guarantee that the printed parts and the verified model can never
/// desync: they did once — the internal branch applied the narrow half-width to
/// the ROOT and the wide one to the TIP, inverting the taper into a sawtooth,
/// while the simulator kept a separate (correct) copy, so the sim passed on a
/// housing it never actually built. `verify_trapezoid_tooth_taper` now guards
/// the taper directly.
///
/// - `external`: teeth point OUTWARD (flexspline) — narrow tip, wide root; the
///   profile carries a leading valley-floor point for a downstream root fillet.
///   `false`: the INTERNAL cavity tooth (circular spline) — WIDE root (outer),
///   NARROW tip (inner).
/// - `thin`: circumferential backlash allowance per flank (mm at the pitch
///   line); pass 0.0 for a zero-backlash reference (the internal member).
pub fn trapezoid_tooth_offsets(
	teeth: usize,
	pitch_r: f64,
	tip_r: f64,
	root_r: f64,
	flank_deg: f64,
	external: bool,
	thin: f64,
) -> Vec<(f64, f64)> {
	let pitch = TAU / teeth as f64;
	let half = pitch / 4.0; // quarter-pitch: the pitch-line tooth half-thickness
	let thin_ang = thin / pitch_r;
	let slope = flank_deg.to_radians().tan() / pitch_r; // angular half-width change per radial mm
	if external {
		// tip narrows outward, root widens inward
		let ht = (half - thin_ang - slope * (tip_r - pitch_r)).max(0.06 / tip_r);
		let hr = half - thin_ang + slope * (pitch_r - root_r);
		vec![(-2.0 * half + hr, root_r), (-hr, root_r), (-ht, tip_r), (ht, tip_r), (hr, root_r)]
	} else {
		// internal cavity: ROOT (outer) is WIDE, TIP (inner) is NARROW
		let ht = half + slope * (root_r - pitch_r);
		let hr = (half - slope * (pitch_r - tip_r)).max(0.06 / tip_r);
		vec![(-ht, root_r), (-hr, tip_r), (hr, tip_r), (ht, root_r)]
	}
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::parts::din6885_key_size;
	use kernel_brep::math::{DAffine3, DVec3};
	use kernel_brep::{intersection, tessellate_adaptive_tol, tessellate_default, validate, volume, VertexId};

	/// A strain-wave trapezoid tooth must TAPER the right way — WIDER at the root
	/// than at the tip (whether the root is the inner radius on the external
	/// flexspline or the outer radius on the internal circular spline). The
	/// sawtooth "thread" that shipped on the harmonic casing was exactly an
	/// INVERTED taper: the internal branch put the narrow half-width at the root
	/// and the wide one at the tip, so the flanks zig-zagged. This guards that
	/// class of bug for BOTH members at the tooth-geometry source, so the printed
	/// parts and the kinematic simulator — which now both call
	/// `trapezoid_tooth_offsets` — can never desync into it again.
	#[test]
	fn trapezoid_tooth_tapers_wide_root_narrow_tip() {
		let module = 0.6;
		for (teeth, tip_r, root_r, external, name) in [
			(54usize, 15.78, 16.74, false, "circular spline (internal)"),
			(52usize, 16.02, 15.06, true, "flexspline (external)"),
		] {
			let pitch_r = module * teeth as f64 / 2.0;
			let offs = trapezoid_tooth_offsets(teeth, pitch_r, tip_r, root_r, 25.0, external, 0.05);
			// widest angular half-extent among the tip points vs the root points
			let tip_hw = offs.iter().filter(|(_, r)| (r - tip_r).abs() < 1e-6).map(|(da, _)| da.abs()).fold(0.0, f64::max);
			let root_hw = offs.iter().filter(|(_, r)| (r - root_r).abs() < 1e-6).map(|(da, _)| da.abs()).fold(0.0, f64::max);
			assert!(
				tip_hw > 0.0 && root_hw > tip_hw,
				"{name}: tooth must be WIDER at the root than the tip (root half-angle {root_hw:.4} > tip {tip_hw:.4}); \
				 an inverted taper (root ≤ tip) is the sawtooth bug"
			);
		}
	}

	/// Validate one gear build: genus-1 watertight — on the default tessellation AND the
	/// adaptive one (the STL export route: a regression guard for FRICTION #6, where
	/// inner-loop caps forced gears through the voxel heal) — volume between the dedendum-
	/// and addendum-cylinder bounds (hole subtracted with safe margins), and the tooth count
	/// visible in the vertex structure (per tooth and per face ring, exactly 4 vertices sit on
	/// the addendum circle: two involute tips + two tip-land samples → 8·z in the solid).
	fn check_gear(label: &str, gear: &Solid, m: f64, z: usize, fw: f64, bore_d: f64, key: Option<KeySize>) {
		let rp = m * z as f64 / 2.0;
		let (ra, rr, rbore) = (rp + m, rp - 1.25 * m, bore_d * 0.5);
		let v = validate(gear);
		let tip_verts = (0..gear.vertex_count() as u32)
			.map(|i| gear.position(VertexId(i)))
			.filter(|p| (p.x * p.x + p.y * p.y).sqrt() >= ra - 1e-6)
			.count();
		// Hole area over-bound: bore circle + a keyway strip running clear across the bore.
		let hole_max = PI * rbore * rbore + key.map_or(0.0, |k| k.b * (k.t2 + rbore));
		// Hole area under-bound: the 48-gon bore polygon (≈ 0.9977 of the circle).
		let hole_min = 0.99 * PI * rbore * rbore;
		let (lo, hi) = ((PI * rr * rr - hole_max) * fw, (PI * ra * ra - hole_min) * fw);
		let vol = volume(gear).abs();
		assert!(
			v.closed
				&& v.manifold && v.genus == 1
				&& tessellate_default(gear).is_watertight()
				&& tessellate_adaptive_tol(gear, 0.01).is_watertight()
				&& tip_verts == 8 * z
				&& vol > lo && vol < hi,
			"{label}: want watertight (default AND adaptive) genus-1, {} tip vertices, volume in ({lo:.0}, {hi:.0}); got {v:?} wt={} adaptive_wt={} tips={tip_verts} vol={vol:.0}",
			8 * z,
			tessellate_default(gear).is_watertight(),
			tessellate_adaptive_tol(gear, 0.01).is_watertight()
		);
	}

	#[test]
	fn module_two_twenty_tooth_gear_with_din6885_keyway_is_valid() {
		// The classic m=2 z=20 α=20° gear (root inside the base circle → radial flank feet),
		// Ø10 bore with the standard 3×3 hub keyway from the DIN 6885 table.
		let (m, z, fw, bore) = (2.0, 20usize, 8.0, 10.0);
		let key = din6885_key_size(bore);
		assert_eq!(key.map(|k| (k.b, k.t2)), Some((3.0, 1.4)), "Ø10 hub takes a 3×3 key, t2=1.4");
		let gear = spur_gear(m, z, fw, bore, 20.0, key);
		check_gear("m2 z20 keyed", &gear, m, z, fw, bore, key);
	}

	#[test]
	fn fine_pitch_forty_eight_tooth_gear_is_valid() {
		// m=1.5 z=48: the root circle lies OUTSIDE the base circle (rr 34.125 > rb 33.829), so
		// the involute starts at the root radius (t_start > 0) — the other generator branch.
		let (m, z, fw, bore) = (1.5, 48usize, 6.0, 8.0);
		let gear = spur_gear(m, z, fw, bore, 20.0, None);
		check_gear("m1.5 z48 plain", &gear, m, z, fw, bore, None);
		// The plain bore is cut with the analytic cylinder primitive, so `exact_volume`
		// recovers the π-exact bore: the 48-gon-vs-circle deficit ((π − 24·sin(2π/48))
		// ·r²·fw ≈ 0.86 mm³) below the faceted volume (FRICTION #15: the bore used to be
		// a raw polygon prism with vol == xvol bit-for-bit). Recovery is conservative,
		// not machine-exact: boolean face recovery splits some bore-wall facets at
		// chord-interior points, and the lens term `Δθ − sin Δθ` is convex, so split
		// pieces under-count slightly — observed 0.015% of the deficit here; we assert
		// ≥ 99% of it is recovered and never more than all of it.
		let (vol, xvol) = (volume(&gear).abs(), kernel_brep::exact_volume(&gear).abs());
		let deficit = (PI - 48.0 * 0.5 * (2.0 * PI / 48.0).sin()) * (bore * 0.5) * (bore * 0.5) * fw;
		let recovered = vol - xvol;
		assert!(
			recovered > 0.99 * deficit && recovered <= deficit + 1e-9,
			"plain spur-gear bore must carry the analytic cylinder tag: want vol − xvol in (0.99, 1]·{deficit:.6}; got vol={vol:.6} xvol={xvol:.6} (recovered {recovered:.6})"
		);
	}

	#[test]
	fn gear_racks_are_exact_basic_rack_bars() {
		// m2 ×100 ×10 @20° (16 whole teeth) and m1 ×30 ×6 @25° (9 teeth): all-planar
		// solids, so the closed form — 1.75m body slab plus n tooth trapezoids — must
		// hold to integration roundoff (1e-7 relative; there is NO faceting term); the
		// crest corners put exactly 4 vertices per tooth on the tip line y = 4m;
		// genus 0 and watertight.
		for (m, len, w, ang) in [(2.0, 100.0, 10.0, 20.0), (1.0, 30.0, 6.0, 25.0)] {
			let rack = gear_rack(m, len, w, ang).expect("valid rack");
			let v = validate(&rack);
			let tan = ang.to_radians().tan();
			let p = PI * m;
			let (w_tip, w_root) = (0.25 * p - m * tan, 0.25 * p + 1.25 * m * tan);
			let n = ((len - 2.0 * w_root) / p).floor() as usize + 1;
			let expected = (len * 1.75 * m + n as f64 * (w_root + w_tip) * 2.25 * m) * w;
			let crest = (0..rack.vertex_count() as u32)
				.map(|i| rack.position(VertexId(i)))
				.filter(|q| (q.y - 4.0 * m).abs() < 1e-9)
				.count();
			let vol = volume(&rack).abs();
			assert!(
				v.closed
					&& v.manifold && v.genus == 0
					&& tessellate_default(&rack).is_watertight()
					&& crest == 4 * n
					&& (vol - expected).abs() / expected < 1e-7,
				"rack m{m} ×{len} @{ang}°: want watertight genus-0 with {} crest verts and exactly {expected:.6} mm³; got {v:?} crest={crest} vol={vol:.6}",
				4 * n
			);
		}
		assert!(
			gear_rack(2.0, 4.0, 10.0, 20.0).is_none()
				&& gear_rack(2.0, 100.0, 10.0, 33.0).is_none()
				&& gear_rack(2.0, f64::NAN, 10.0, 20.0).is_none(),
			"a bar too short for one whole tooth, a 33° pressure angle (root corners merge) and NaN must be refused"
		);
	}

	#[test]
	fn internal_gears_are_valid_rings_and_mesh_their_conjugate_pinion_cleanly() {
		// m2 z36 rim Ø84 (ring tip 34 > base 33.83 → t_start > 0) and m1.5 z24 rim Ø44
		// (tip 16.5 < base 16.91 → the radial-foot branch): genus-1 watertight rings
		// spanning r_tip … rim/2, volume strictly between the to-the-root and
		// to-the-tip annuli. The boolean bore cut may add cap-loop vertices ON the
		// tooth polygon's chords (collinear arrangement debris, shape unchanged), so
		// r_min sits within one chord sagitta below r_tip — the outline's angular step
		// is at most a quarter pitch, bounding the sagitta by r_tip·(1 − cos(π/4z));
		// r_max stays exactly rim/2 (debris on rim chords only dips inward).
		for (m, z, fw, rim) in [(2.0, 36usize, 8.0, 84.0), (1.5, 24usize, 6.0, 44.0)] {
			let ring = internal_gear(m, z, fw, rim, 20.0).expect("valid ring");
			let v = validate(&ring);
			let rp = m * z as f64 / 2.0;
			let (r_root, r_tip, rim_r) = (rp + 1.25 * m, rp - m, rim * 0.5);
			let rr: Vec<f64> = (0..ring.vertex_count() as u32)
				.map(|i| {
					let q = ring.position(VertexId(i));
					(q.x * q.x + q.y * q.y).sqrt()
				})
				.collect();
			let (r_min, r_max) = rr.iter().fold((f64::INFINITY, 0.0_f64), |(lo, hi), &r| (lo.min(r), hi.max(r)));
			let (lo, hi) = ((0.99 * PI * rim_r * rim_r - PI * r_root * r_root) * fw, (PI * rim_r * rim_r - 0.99 * PI * r_tip * r_tip) * fw);
			let vol = volume(&ring).abs();
			let sagitta = r_tip * (1.0 - (PI / (4.0 * z as f64)).cos());
			assert!(
				v.closed
					&& v.manifold && v.genus == 1
					&& tessellate_default(&ring).is_watertight()
					&& tessellate_adaptive_tol(&ring, 0.01).is_watertight()
					&& r_min > r_tip - sagitta
					&& r_min < r_tip + 1e-9
					&& (r_max - rim_r).abs() < 1e-9
					&& vol > lo && vol < hi,
				"internal gear m{m} z{z}: want watertight (default AND adaptive — the FRICTION #6 export route) genus-1 spanning r ({:.4}…{r_tip}]–{rim_r}, volume in ({lo:.0}, {hi:.0}); got {v:?} adaptive_wt={} r=[{r_min:.4},{r_max:.4}] vol={vol:.0}",
				r_tip - sagitta,
				tessellate_adaptive_tol(&ring, 0.01).is_watertight()
			);
		}

		// Conjugacy proof: the z36 ring meshed with its z18 pinion at the standard
		// centre distance (36 − 18)·2/2 = 18 — pinion tooth 0 centred in ring space 0,
		// tip circles interleaving by the full 2m = 4 mm working depth (pinion crest
		// reaches r38 inside the ring-tip r34) — must have an EMPTY exact boolean
		// intersection: conjugate involute flanks touch but never cross, and the
		// chordal faceting nets a few µm of clearance (the smaller pinion is shaved
		// more than the ring protrudes). A flank-math error of even a fraction of a
		// degree would overlap by whole mm³. The pinion is z-shifted 0.37 so the end
		// caps are not coplanar — flank conjugacy is what is probed here.
		let ring = internal_gear(2.0, 36, 8.0, 84.0, 20.0).expect("valid ring");
		let pinion = spur_gear(2.0, 18, 8.0, 8.0, 20.0, None).transformed(DAffine3::from_translation(DVec3::new(18.0, 0.0, 0.37)));
		let clash = intersection(&ring, &pinion);
		let clash_vol = if clash.face_count() == 0 { 0.0 } else { volume(&clash).abs() };
		assert!(
			(18.0 + (2.0 * (18.0 / 2.0 + 1.0)) - (36.0 - 2.0)) == 4.0 && clash_vol < 0.01,
			"z36 ring × z18 pinion at C=18: tip circles interleave 4 mm yet the mesh must not interpenetrate; got intersection volume {clash_vol:.4} mm³"
		);
		assert!(
			internal_gear(2.0, 36, 8.0, 70.0, 20.0).is_none() && internal_gear(2.0, 36, 8.0, 84.0, 30.0).is_none(),
			"a rim thinner than the root circle (Ø70 < Ø77) and a 30° ring (root land pinches shut) must be refused"
		);
	}

	#[test]
	fn positive_profile_shift_grows_tip_and_thickens_pitch_tooth() {
		// The PLAN-26 stage-B move: a +0.3 profile shift on the 11T pinion
		// (module 0.7543, 27° PA, printable 0.05 mm/flank thinning) must make the
		// tip circle STRICTLY larger (ra grows by exactly x·m) AND the tooth chord
		// MEASURED where the two flanks cross the pitch circle STRICTLY thicker
		// (each flank half-thickness grows by x·m·tan α) — the strength gain a
		// shift buys, read straight off the generated polygon.
		let (m, z, pa, thin, x) = (0.7542857142857143_f64, 11usize, 27.0_f64, 0.05, 0.3);
		let rp = m * z as f64 / 2.0;
		let base = involute_ring_outline_shifted(m, z, pa, true, false, thin, 0.0).expect("x=0");
		let shifted = involute_ring_outline_shifted(m, z, pa, true, false, thin, x).expect("x=+0.3");
		let tip = |o: &[DVec2]| o.iter().map(|p| (p.x * p.x + p.y * p.y).sqrt()).fold(0.0_f64, f64::max);
		// tooth-0 (centred on +X) chord at the pitch circle: the two flanks each
		// cross r = rp once; |polar angle| < π/z isolates tooth 0's two crossings.
		let pitch_chord = |o: &[DVec2]| -> f64 {
			let n = o.len();
			let mut cx: Vec<DVec2> = Vec::new();
			for i in 0..n {
				let (a, b) = (o[i], o[(i + 1) % n]);
				let (ra, rb) = ((a.x * a.x + a.y * a.y).sqrt(), (b.x * b.x + b.y * b.y).sqrt());
				if (ra - rp) * (rb - rp) < 0.0 {
					let t = (rp - ra) / (rb - ra);
					let p = DVec2::new(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t);
					if f64::atan2(p.y, p.x).abs() < PI / z as f64 {
						cx.push(p);
					}
				}
			}
			assert_eq!(cx.len(), 2, "the pitch circle must cross tooth 0 exactly twice; got {}", cx.len());
			((cx[1].x - cx[0].x).powi(2) + (cx[1].y - cx[0].y).powi(2)).sqrt()
		};
		let (t0, ts) = (tip(&base), tip(&shifted));
		let (c0, cs) = (pitch_chord(&base), pitch_chord(&shifted));
		let exp_tip = x * m; // ra = rp + m(1+x): tip grows by exactly x·m
		assert!(
			ts > t0 + 0.999 * exp_tip && ts < t0 + 1.001 * exp_tip && cs > c0,
			"x=+{x} on 11T: tip radius {t0:.4}→{ts:.4} (want +{exp_tip:.4} = x·m) and pitch-line tooth chord {c0:.4}→{cs:.4} (want strictly thicker)"
		);
	}

	/// The **root fillet** (the highest-value printed-gear strength feature, added 2026-07-11
	/// on the drive-family finding that sharp-root generators sit far below handbook Lewis
	/// ratings). Four falsifiable facts on one m=0.6 z=18 gear (the module-0.6, root-inside-base
	/// regime of the CYCLO/PLAN drives):
	///  1. **coeff 0 is byte-identical** to the sharp `spur_gear` — every existing caller frozen;
	///  2. the filleted solid is still a **valid genus-1 gear** with the SAME tip radius (the
	///     fillet touches only the root, never the addendum);
	///  3. the fillet **adds material at the root** — the profile gains vertices at radii in the
	///     open band (rr, rft] that the sharp outline never visits, and the sharp radial-foot
	///     corner at exactly rr is gone (the minimum-radius points are now the fillet–root
	///     tangencies, one per tooth, not the old full root-land arc);
	///  4. the tangency is **geometrically exact** — every fillet point lies on a circle of
	///     radius rf about a centre at radius rr+rf (residual < 1e-9 mm), i.e. a true circular
	///     arc tangent to the root circle, not a chamfer.
	#[test]
	fn root_fillet_rounds_the_foot_adds_root_material_and_leaves_coeff0_byte_identical() {
		let (m, z, alpha) = (0.6, 18usize, 20.0_f64.to_radians());
		let (rp, ra, rr) = (m * z as f64 / 2.0, m * z as f64 / 2.0 + m, m * z as f64 / 2.0 - 1.25 * m);
		let rb = rp * alpha.cos();
		assert!(rr < rb, "test premise: this gear has radial feet (rr {rr:.3} < rb {rb:.3})");

		// (1) coeff 0 byte-identical to the historical sharp generator.
		let sharp = involute_outline_df(m, z, alpha, ra, rr, 0.0, 0.0);
		let sharp_ref = involute_outline(m, z, alpha, ra, rr);
		assert_eq!(sharp, sharp_ref, "coeff-0 filleted path must equal the sharp outline exactly");

		let rf = 0.3 * m; // 0.18 mm — a typical printable root fillet
		let fil = involute_outline_df(m, z, alpha, ra, rr, 0.0, rf);
		let rad = |p: &DVec2| (p.x * p.x + p.y * p.y).sqrt();
		let rft = (rr * rr + 2.0 * rr * rf).sqrt();

		// (2) still a valid genus-1 solid with the tip radius untouched.
		let gear = spur_gear_filleted(m, z, 5.0, 5.0, 20.0, None, 0.3);
		let v = validate(&gear);
		let tip_sharp = involute_outline(m, z, alpha, ra, rr).iter().map(rad).fold(0.0, f64::max);
		let tip_fil = fil.iter().map(rad).fold(0.0_f64, f64::max);
		assert!(
			v.is_valid() && v.genus == 1 && (tip_fil - tip_sharp).abs() < 1e-9,
			"filleted gear must stay a valid genus-1 solid with an unchanged tip radius \
			 (valid {}, genus {}, tip {tip_sharp:.4}→{tip_fil:.4})",
			v.is_valid(),
			v.genus
		);

		// (3) material added at the root AND the sharp radial-foot step is broken up. The sharp
		// outline steps from the root land (rr) straight to the flank start (rb) at one angle —
		// a ~(rb−rr) radial jump between consecutive points. The fillet fills that band, so the
		// largest consecutive radial jump in the root region (r < rb) collapses, no point dips
		// below rr, and the filleted outline carries strictly more vertices.
		let min_r = fil.iter().map(rad).fold(f64::INFINITY, f64::min);
		let in_band = fil.iter().filter(|p| rad(p) > rr + 1e-6 && rad(p) < rft + 1e-6).count();
		let root_step = |o: &[DVec2]| -> f64 {
			let n = o.len();
			(0..n)
				.filter(|&i| rad(&o[i]) < rb + 1e-9 && rad(&o[(i + 1) % n]) < rb + 1e-9)
				.map(|i| (rad(&o[i]) - rad(&o[(i + 1) % n])).abs())
				.fold(0.0_f64, f64::max)
		};
		let (step_sharp, step_fil) = (root_step(&sharp), root_step(&fil));
		assert!(
			min_r > rr - 1e-9 && in_band >= z && fil.len() > sharp.len() && step_fil < 0.5 * step_sharp,
			"root fillet must add root material and smooth the foot: min radius {min_r:.4} ≥ rr {rr:.4}; \
			 {in_band} fillet points in (rr,rft) (≥{z}); vertices {}→{}; worst root radial step {step_sharp:.4}→{step_fil:.4} mm (want < half)",
			sharp.len(),
			fil.len()
		);

		// (4) the fillet is a TRUE circular arc tangent to the root circle — every in-band fillet
		// point lies within 1e-9 mm of radius rf from one of the 2·z reconstructed fillet centres
		// (each at radius rr+rf, polar angle foot±δ), never a chamfer.
		let delta = (rf / (rr + rf)).asin();
		let half = PI / (2.0 * z as f64) + (alpha.tan() - alpha);
		let (pitch, mut centers) = (2.0 * PI / z as f64, Vec::new());
		for k in 0..z {
			let a_l = k as f64 * pitch - half;
			let g0 = a_l - (pitch - 2.0 * half);
			for pc in [g0 + delta, a_l - delta] {
				centers.push(DVec2::new((rr + rf) * pc.cos(), (rr + rf) * pc.sin()));
			}
		}
		let worst = fil
			.iter()
			.filter(|p| rad(p) > rr + 1e-6 && rad(p) < rft - 1e-6)
			.map(|p| {
				centers
					.iter()
					.map(|c| (((p.x - c.x).powi(2) + (p.y - c.y).powi(2)).sqrt() - rf).abs())
					.fold(f64::INFINITY, f64::min)
			})
			.fold(0.0_f64, f64::max);
		assert!(worst < 1e-9, "every fillet point must lie exactly on its rf-radius arc about a centre at rr+rf; worst residual {worst:.2e} mm");
	}
}
