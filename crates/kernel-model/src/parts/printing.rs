// Copyright (c) LMCAD. Licensed under the MIT License.

//! **3D-printing-native hole variants**: the two standard tricks for printing
//! holes without support material, as parts-level cuts (they compose the same
//! kernel drills/booleans as everything else — the hole wizard itself stays
//! untouched):
//!
//! - [`teardrop_hole`] — a horizontal hole whose crown is extended to a 45°/45°
//!   teardrop apex along the build direction, so no part of the bore overhangs
//!   more than 45°;
//! - [`bridged_counterbore`] — a DIN 974 counterbore whose small hole is **left
//!   sealed by a thin sacrificial bridge layer**: the counterbore's ceiling
//!   prints as a flat bridge instead of sagging into the void, and you drill the
//!   membrane out afterwards. The as-printed solid is deliberately NOT a through
//!   hole (genus unchanged) — that is the whole point, and the tests assert it.
//!
//! Both follow the **hole-wizard convention**: `at` on the entry face, `axis`
//! pointing INTO the material, cutters overshooting the faces so no coplanar
//! membranes are left (except the one the bridge intends).

use kernel_brep::holes::metric_hole_spec;
use kernel_brep::math::{DAffine3, DMat3, DVec2, DVec3};
use kernel_brep::{cylinder, difference, extrude, Solid};

/// The teardrop outline for a bore of radius `r`: the 48-gon arc over the lower
/// 270° plus the two 45° tangent roof lines meeting at the apex `√2·r` above
/// centre, wound CCW in the `(across, up)` plane.
fn teardrop_outline(r: f64) -> Vec<DVec2> {
	let mut poly: Vec<DVec2> = (0..=36)
		.map(|i| {
			let a = (135.0 + 7.5 * i as f64).to_radians(); // 135° → 405°(=45°), through the bottom
			DVec2::new(r * a.cos(), r * a.sin())
		})
		.collect();
	poly.push(DVec2::new(0.0, r * std::f64::consts::SQRT_2));
	poly
}

/// Cut a **teardrop hole** — the printable form of a horizontal bore: a Ø`d`
/// hole through `through` mm of material whose crown continues past the circle
/// as two 45° roof lines meeting at an apex `√2·d/2` above the centre along the
/// build direction, so the printer never bridges an overhang steeper than 45°
/// and the hole needs no support. `at` on the entry face, `axis` INTO the
/// material (the hole-wizard convention), `up` the build (+Z of the print bed)
/// direction — any vector not parallel to `axis`; its in-plane component is
/// used. Adds one tunnel (genus +1). Print-shop honesty: the bore is the exact
/// nominal circle over its lower 270° — the teardrop only ADDS clearance above,
/// so pins/bolts still locate on the lower arc; size holes for fit with the
/// usual ISO 286 allowances. `None` for a degenerate axis, `up` parallel to
/// `axis`, or non-positive `d`/`through`.
pub fn teardrop_hole(solid: &Solid, at: DVec3, axis: DVec3, up: DVec3, d: f64, through: f64) -> Option<Solid> {
	let a = axis.try_normalize()?;
	let u = (up - a * up.dot(a)).try_normalize()?;
	if !(d > 0.0 && d.is_finite() && through > 0.0 && through.is_finite()) {
		return None;
	}
	let e1 = u.cross(a); // (e1, u, a) right-handed: the outline's +y is the build up
	let cutter = extrude(&teardrop_outline(d * 0.5), through + 1.0)
		.transformed(DAffine3::from_mat3_translation(DMat3::from_cols(e1, u, a), at - a * 0.5));
	Some(difference(solid, &cutter))
}

/// Cut a **sacrificial-bridge counterbore** for an M-`m` cap screw (M2–M12, the
/// same DIN 974-1 / ISO 273 table as the hole wizard's `counterbore_hole`): the
/// counterbore pocket sinks from the face, but the medium-fit clearance bore is
/// started only `bridge` mm BELOW the pocket floor — leaving a thin printable
/// membrane that bridges the counterbore ceiling flat. Print with the pocket
/// facing down/outward, then **drill the membrane out** (the echo of the bore Ø
/// is in the table); one printer layer height (0.2–0.3) is the usual `bridge`.
/// The as-printed solid is intentionally NOT a through hole: genus is
/// **unchanged** (the test asserts genus 0 against the wizard's genus 1).
/// `at` on the entry face, `axis` INTO the material, `through` the total
/// material depth. `None` outside the M-table, for a degenerate axis, a
/// non-positive `bridge`, or when pocket + bridge don't fit inside `through`.
pub fn bridged_counterbore(solid: &Solid, at: DVec3, axis: DVec3, m: f64, through: f64, bridge: f64) -> Option<Solid> {
	let spec = metric_hole_spec(m)?;
	let a = axis.try_normalize()?;
	let (cb_d, cb_t) = (spec.counterbore_d, spec.counterbore_depth);
	if !(bridge > 0.0 && through > cb_t + bridge && through.is_finite() && bridge.is_finite()) {
		return None;
	}
	let (e1, e2) = super::perp_basis(a);
	let frame = DMat3::from_cols(e1, e2, a);
	// Counterbore pocket: 1 mm proud of the face down to the table depth.
	let pocket = cylinder(DVec3::ZERO, DVec3::Z, cb_d * 0.5, cb_t + 1.0, 48).transformed(DAffine3::from_mat3_translation(frame, at - a));
	// Clearance bore: starts `bridge` below the pocket floor, exits 1 mm past the
	// far face — the membrane in between is the sacrificial bridge.
	let bore_start = cb_t + bridge;
	let bore = cylinder(DVec3::ZERO, DVec3::Z, spec.clearance[1] * 0.5, through + 1.0 - bore_start, 48)
		.transformed(DAffine3::from_mat3_translation(frame, at + a * bore_start));
	Some(difference(&difference(solid, &pocket), &bore))
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
			&& v.manifold
			&& v.genus == want_genus
			&& tessellate_default(s).is_watertight()
			&& tessellate_adaptive_tol(s, 0.01).is_watertight();
		(ok, format!("{v:?} wt={} adaptive_wt={}", tessellate_default(s).is_watertight(), tessellate_adaptive_tol(s, 0.01).is_watertight()))
	}

	#[test]
	fn teardrop_holes_roof_at_45_degrees_and_keep_the_exact_lower_arc() {
		// A 20-cube gets a Ø8 teardrop along −Y (up = +Z) and a Ø5 along −X with a
		// slanted `up` whose in-plane part is +Z: genus 2, apex vertices at exactly
		// √2·r along up, and volume = cube − the two exact teardrop prisms
		// (270° 48-gon arc + roof triangles = 18r²·sin7.5° + r² each) to 1e-6.
		let cube = cuboid(DVec3::new(-10.0, -10.0, -10.0), DVec3::new(10.0, 10.0, 10.0));
		let one = teardrop_hole(&cube, DVec3::new(0.0, 10.0, -4.0), -DVec3::Y, DVec3::Z, 8.0, 20.0).expect("Ø8 teardrop");
		let two = teardrop_hole(&one, DVec3::new(10.0, 0.0, 5.0), -DVec3::X, DVec3::new(-0.4, 0.0, 1.0), 5.0, 20.0)
			.expect("Ø5 teardrop, up orthogonalised");
		let (ok, diag) = check(&two, 2);
		let area = |r: f64| 18.0 * r * r * (7.5_f64.to_radians()).sin() + r * r;
		let expected = 8000.0 - (area(4.0) + area(2.5)) * 20.0;
		let vol = volume(&two).abs();
		let apex8 = (0..two.vertex_count() as u32)
			.map(|i| two.position(VertexId(i)))
			.filter(|p| p.x.abs() < 1e-9 && (p.z - (-4.0 + 4.0 * 2.0_f64.sqrt())).abs() < 1e-9)
			.count();
		let apex5 = (0..two.vertex_count() as u32)
			.map(|i| two.position(VertexId(i)))
			.filter(|p| p.y.abs() < 1e-9 && (p.z - (5.0 + 2.5 * 2.0_f64.sqrt())).abs() < 1e-9)
			.count();
		assert!(
			ok && apex8 >= 2
				&& apex5 >= 2 && (vol - expected).abs() / expected < 1e-6
				&& teardrop_hole(&cube, DVec3::new(0.0, 10.0, 0.0), -DVec3::Y, DVec3::Y, 8.0, 20.0).is_none(),
			"teardrops: want watertight×2 genus-2, apex lines at √2·r up, exactly {expected:.3}mm³ (and refusal when up ∥ axis); got {diag} apex8={apex8} apex5={apex5} vol={vol:.3}"
		);
	}

	#[test]
	fn bridged_counterbore_seals_the_bore_until_you_drill_it() {
		// M5 into a 30 × 30 × 10 plate, 0.3 bridge: the DIN 974 Ø10 × 5.8 pocket and
		// the Ø5.5 bore stopping 0.3 under it — genus STAYS 0 (the membrane seals;
		// that is the printable point) while the hole wizard's counterbore_hole on
		// the same spot gives genus 1; volume = plate − pocket − bore·3.9 within 1%;
		// the membrane's underside (bore top) sits at exactly z = 10 − 5.8 − 0.3.
		// Refusals: M7, zero bridge, and a bridge past the material.
		let plate = cuboid(DVec3::new(-15.0, -15.0, 0.0), DVec3::new(15.0, 15.0, 10.0));
		let at = DVec3::new(0.0, 0.0, 10.0);
		let bridged = bridged_counterbore(&plate, at, -DVec3::Z, 5.0, 10.0, 0.3).expect("M5 bridged");
		let wizard = kernel_brep::holes::counterbore_hole(&plate, at, -DVec3::Z, 5.0, kernel_brep::holes::Fit::Medium, None)
			.expect("wizard counterbore");
		let (ok, diag) = check(&bridged, 0);
		let wizard_genus = validate(&wizard).genus;
		let floor = (0..bridged.vertex_count() as u32)
			.map(|i| bridged.position(VertexId(i)))
			.filter(|p| (p.x * p.x + p.y * p.y).sqrt() < 5.0 + 1e-9 && p.z > 0.5)
			.map(|p| p.z)
			.fold(f64::INFINITY, f64::min);
		let expected = 30.0 * 30.0 * 10.0 - PI * 5.0 * 5.0 * 5.8 - PI * 2.75 * 2.75 * (10.0 - 5.8 - 0.3);
		let vol = volume(&bridged).abs();
		assert!(
			ok && wizard_genus == 1
				&& (floor - (10.0 - 5.8 - 0.3)).abs() < 1e-9
				&& (vol - expected).abs() / expected < 0.01
				&& bridged_counterbore(&plate, at, -DVec3::Z, 7.0, 10.0, 0.3).is_none()
				&& bridged_counterbore(&plate, at, -DVec3::Z, 5.0, 10.0, 0.0).is_none()
				&& bridged_counterbore(&plate, at, -DVec3::Z, 5.0, 6.0, 0.3).is_none(),
			"bridged M5: want watertight×2 genus-0 (vs wizard genus-1), bore top at z=3.9, ~{expected:.0}mm³; got {diag} wizard_genus={wizard_genus} bore_top={floor} vol={vol:.0}"
		);
	}
}
