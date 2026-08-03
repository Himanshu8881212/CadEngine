// Copyright (c) LMCAD. Licensed under the MIT License.

//! **Motor interfaces**: NEMA 17/23 stepper frame dimensions — simplified motor
//! bodies for assembly/clearance work, mount plates, and the mount **feature cut**
//! (pilot bore + bolt pattern machined into any face) — plus hobby-servo pockets
//! (SG90 / MG996R). The NEMA numbers are the ICS 16 frame dimensions (inch values
//! converted, cited at the table); servo dimensions are the de-facto datasheet
//! values reproduced across the TowerPro-pattern clones, cited at the table.

use kernel_brep::geom::perp_basis;
use kernel_brep::holes::{clearance_hole, drill, Fit, HoleDepth};
use kernel_brep::math::{DAffine3, DMat3, DVec2, DVec3};
use kernel_brep::{cuboid, cylinder, difference, union, Solid};

/// One NEMA stepper frame row (all mm).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NemaSpec {
	/// Frame number (17, 23).
	pub frame: usize,
	/// Faceplate width across the square body.
	pub face_w: f64,
	/// Corner chamfer of the square body (45°, across the corner).
	pub corner: f64,
	/// Mounting bolt square spacing (hole-centre to hole-centre).
	pub bolt_spacing: f64,
	/// Mounting screw metric size (NEMA 23's imperial #10-32 is stocked as M5
	/// in the metric ecosystem — de-facto, documented).
	pub bolt_m: f64,
	/// Pilot (register) boss diameter.
	pub pilot_d: f64,
	/// Pilot boss height above the faceplate.
	pub pilot_h: f64,
	/// Output shaft diameter.
	pub shaft_d: f64,
	/// Output shaft length beyond the faceplate.
	pub shaft_len: f64,
}

/// NEMA frame table. Source: NEMA ICS 16 stepper frame dimensions as published
/// across stepper datasheets (inch → mm): NEMA 17 — face 1.67″ ≈ 42.3, bolts
/// 1.220″ = 31.0 square M3, pilot 0.866″ = 22.0 × 2.0 proud, shaft Ø5 × 24;
/// NEMA 23 — face 2.22″ ≈ 56.4, bolts 1.856″ = 47.14 square (#10-32 → M5
/// de-facto), pilot 1.500″ = 38.1 × 1.6 proud, shaft Ø0.250″ = 6.35 × 21.
const NEMA: [NemaSpec; 2] = [
	NemaSpec { frame: 17, face_w: 42.3, corner: 5.0, bolt_spacing: 31.0, bolt_m: 3.0, pilot_d: 22.0, pilot_h: 2.0, shaft_d: 5.0, shaft_len: 24.0 },
	NemaSpec { frame: 23, face_w: 56.4, corner: 6.0, bolt_spacing: 47.14, bolt_m: 5.0, pilot_d: 38.1, pilot_h: 1.6, shaft_d: 6.35, shaft_len: 21.0 },
];

/// The NEMA frame row for `frame` ∈ {17, 23}, or `None`.
pub fn nema_spec(frame: usize) -> Option<NemaSpec> {
	NEMA.iter().find(|s| s.frame == frame).copied()
}

/// The chamfered-corner square faceplate outline (a 45°-cornered octagon), CCW.
fn nema_outline(face_w: f64, corner: f64) -> Vec<DVec2> {
	let h = face_w * 0.5;
	let c = corner * 0.5; // half the across-corner chamfer per edge
	vec![
		DVec2::new(h, h - c),
		DVec2::new(h - c, h),
		DVec2::new(-(h - c), h),
		DVec2::new(-h, h - c),
		DVec2::new(-h, -(h - c)),
		DVec2::new(-(h - c), -h),
		DVec2::new(h - c, -h),
		DVec2::new(h, -(h - c)),
	]
}

/// A **simplified NEMA stepper motor body** for assembly, mounting and clearance
/// work: the chamfered-corner square body extruded `body_len` *below* the
/// faceplate (face at z = 0, body in −z), the pilot register boss and the output
/// shaft along +Z. Frame numbers 17 and 23; typical body lengths 34/40/48 (N17)
/// and 41/56/76 (N23) — any positive length builds. Genus 0; unions are
/// transverse (boss and shaft root inside the body, no coplanar caps). This is
/// the **envelope**, honestly simplified: no wiring box, end-bell ribs, rear
/// shaft or label recess. `None` for an unknown frame or degenerate length.
pub fn nema_motor(frame: usize, body_len: f64) -> Option<Solid> {
	let s = nema_spec(frame)?;
	if !(body_len > 0.0 && body_len.is_finite()) {
		return None;
	}
	let body = kernel_brep::extrude(&nema_outline(s.face_w, s.corner), -body_len);
	// Boss and shaft both root 1 mm inside the body so every union contact is a
	// wall crossing a cap transversely, never cap-on-cap.
	let boss = cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, s.pilot_d * 0.5, s.pilot_h + 1.0, 48);
	let shaft = cylinder(DVec3::new(0.0, 0.0, -2.0), DVec3::Z, s.shaft_d * 0.5, s.shaft_len + 2.0, 48);
	Some(union(&union(&body, &boss), &shaft))
}

/// Cut a **NEMA mount** into a face: the pilot through-bore (register fit
/// `pilot_d + 0.2`) plus the four ISO 273 medium clearance holes on the frame's
/// bolt square, all through `through` mm of material. `at` is the motor axis on
/// the face, `axis` the outward face normal; the bolt square is aligned to the
/// face frame of `axis` (`perp_basis` — for the world ±Z that is the world X/Y
/// axes). `None` for an unknown frame, a degenerate axis or a non-positive span.
pub fn nema_mount_cut(solid: &Solid, at: DVec3, axis: DVec3, frame: usize, through: f64) -> Option<Solid> {
	let s = nema_spec(frame)?;
	let axis = axis.try_normalize()?;
	if !(through > 0.0 && through.is_finite()) {
		return None;
	}
	let (e1, e2) = perp_basis(axis);
	let mut cut = drill(solid, at, -axis, s.pilot_d + 0.2, HoleDepth::Through(through), Some(48)).ok()?;
	let half = s.bolt_spacing * 0.5;
	for (sx, sy) in [(1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)] {
		let p = at + e1 * (sx * half) + e2 * (sy * half);
		cut = clearance_hole(&cut, p, -axis, s.bolt_m, Fit::Medium, None).ok()?;
	}
	Some(cut)
}

/// A square **NEMA mount plate**: `face_w + 2·margin` on a side, `thickness`
/// thick (z 0…thickness), with the [`nema_mount_cut`] pattern through it (pilot
/// register bore + four clearance holes). The minimal motor bracket — print it,
/// or use it as stock for further features. Genus 5 (five through-holes). `None`
/// for an unknown frame or degenerate dimensions (margin ≥ 0).
pub fn nema_mount_plate(frame: usize, thickness: f64, margin: f64) -> Option<Solid> {
	let s = nema_spec(frame)?;
	if !(thickness > 0.0 && thickness.is_finite() && margin >= 0.0 && margin.is_finite()) {
		return None;
	}
	let half = s.face_w * 0.5 + margin;
	let plate = cuboid(DVec3::new(-half, -half, 0.0), DVec3::new(half, half, thickness));
	nema_mount_cut(&plate, DVec3::new(0.0, 0.0, thickness), DVec3::Z, frame, thickness)
}

/// One hobby-servo size row (all mm): case footprint and the ear screw pattern.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ServoSpec {
	/// Catalog name (`"sg90"`, `"mg996r"`).
	pub name: &'static str,
	/// Case length (the long side, excluding the mounting ears).
	pub body_l: f64,
	/// Case width.
	pub body_w: f64,
	/// Screw-hole spacing along the long axis (hole-centre to hole-centre).
	pub hole_pitch: f64,
	/// Screw-hole row spacing across the short axis (0 = single in-line pair).
	pub hole_row: f64,
	/// Pilot diameter drilled for the self-tapping mounting screws.
	pub pilot_d: f64,
}

/// Servo table. Source: the TowerPro datasheet dimensions reproduced across the
/// clone ecosystem — SG90: case 23.0 × 12.2, two in-line ear holes 27.5 apart
/// (Ø2 screws → Ø1.8 pilot); MG996R: case 40.7 × 19.7, four ear holes on a
/// 49.5 × 10.0 rectangle (Ø4 screws → Ø3.5 pilot).
const SERVOS: [ServoSpec; 2] = [
	ServoSpec { name: "sg90", body_l: 23.0, body_w: 12.2, hole_pitch: 27.5, hole_row: 0.0, pilot_d: 1.8 },
	ServoSpec { name: "mg996r", body_l: 40.7, body_w: 19.7, hole_pitch: 49.5, hole_row: 10.0, pilot_d: 3.5 },
];

/// The servo row for a catalog `name` (`"sg90"`, `"mg996r"`), or `None`.
pub fn servo_spec(name: &str) -> Option<ServoSpec> {
	SERVOS.iter().find(|s| s.name.eq_ignore_ascii_case(name)).copied()
}

/// Cut a **hobby-servo pocket** through a panel: the rectangular case cutout
/// (case + 0.4 mm fit clearance, long side along the face frame's first axis)
/// plus the ear-screw pilot holes, all through `through` mm of material — the
/// drop-in servo mount of every printed pan/tilt and RC part. `at` is the pocket
/// centre on the face, `axis` the outward normal; orientation follows
/// `perp_basis(axis)` like [`nema_mount_cut`]. SG90 pockets get the in-line pilot
/// pair, MG996R the four-hole rectangle. The servo's wire-exit notch is NOT cut
/// (vendor-specific; add a `drill`/box cut where yours needs it). `None` for an
/// unknown name, a degenerate axis or a non-positive span.
pub fn servo_pocket(solid: &Solid, at: DVec3, axis: DVec3, name: &str, through: f64) -> Option<Solid> {
	let s = servo_spec(name)?;
	let axis = axis.try_normalize()?;
	if !(through > 0.0 && through.is_finite()) {
		return None;
	}
	let (e1, e2) = perp_basis(axis);
	let (hl, hw) = (s.body_l * 0.5 + 0.2, s.body_w * 0.5 + 0.2);
	// Case cutout: a local-frame box overshooting 1 mm both sides of the panel.
	let pocket = cuboid(DVec3::new(-hl, -hw, -(through + 1.0)), DVec3::new(hl, hw, 1.0))
		.transformed(DAffine3::from_mat3_translation(DMat3::from_cols(e1, e2, axis), at));
	let mut cut = difference(solid, &pocket);
	let stations: Vec<DVec3> = if s.hole_row == 0.0 {
		vec![at + e1 * (s.hole_pitch * 0.5), at - e1 * (s.hole_pitch * 0.5)]
	} else {
		[(1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)]
			.iter()
			.map(|(sx, sy)| at + e1 * (sx * s.hole_pitch * 0.5) + e2 * (sy * s.hole_row * 0.5))
			.collect()
	};
	for p in stations {
		cut = drill(&cut, p, -axis, s.pilot_d, HoleDepth::Through(through), None).ok()?;
	}
	Some(cut)
}

#[cfg(test)]
mod tests {
	use super::*;
	use kernel_brep::{tessellate_adaptive_tol, tessellate_default, validate, volume, VertexId};
	use std::f64::consts::PI;

	/// Area of the chamfered-square outline: square minus the four corner triangles.
	fn outline_area(face_w: f64, corner: f64) -> f64 {
		face_w * face_w - 4.0 * 0.5 * (corner * 0.5) * (corner * 0.5)
	}

	#[test]
	fn nema_table_matches_the_published_frame_dimensions() {
		// The ICS 16 anchor values (inch-converted): N17 31.0 bolt square / Ø22 pilot
		// / Ø5 shaft; N23 47.14 / Ø38.1 / Ø6.35. NEMA 8 is not in the table.
		let probe = |f: usize| nema_spec(f).map(|s| (s.bolt_spacing, s.pilot_d, s.shaft_d, s.bolt_m));
		assert_eq!(
			[probe(17), probe(23), probe(8)],
			[Some((31.0, 22.0, 5.0, 3.0)), Some((47.14, 38.1, 6.35, 5.0)), None],
			"NEMA frame table anchors"
		);
	}

	#[test]
	fn nema_motor_bodies_are_watertight_envelopes_of_the_frame_volume() {
		// N17 ×40 and N23 ×56: genus-0, watertight on both routes, spanning exactly
		// −body_len…+shaft_len in z and the face width across, volume within 1% of
		// the closed form (chamfered prism + pilot ring + shaft, transverse-union
		// overlaps counted once; 48-gon ≈ 0.23% under the π terms).
		for (frame, len) in [(17usize, 40.0), (23usize, 56.0)] {
			let s = nema_spec(frame).expect("frame row");
			let m = nema_motor(frame, len).expect("valid body");
			let v = validate(&m);
			let (mut zmin, mut zmax, mut xmax) = (f64::INFINITY, f64::NEG_INFINITY, 0.0_f64);
			for i in 0..m.vertex_count() as u32 {
				let p = m.position(VertexId(i));
				zmin = zmin.min(p.z);
				zmax = zmax.max(p.z);
				xmax = xmax.max(p.x.abs());
			}
			let expected = outline_area(s.face_w, s.corner) * len
				+ PI * (s.pilot_d * 0.5).powi(2) * s.pilot_h
				+ PI * (s.shaft_d * 0.5).powi(2) * (s.shaft_len - s.pilot_h);
			let vol = volume(&m).abs();
			assert!(
				v.closed
					&& v.manifold && v.genus == 0
					&& tessellate_default(&m).is_watertight()
					&& tessellate_adaptive_tol(&m, 0.01).is_watertight()
					&& (zmin + len).abs() < 1e-9
					&& (zmax - s.shaft_len).abs() < 1e-9
					&& (xmax - s.face_w * 0.5).abs() < 1e-9
					&& (vol - expected).abs() / expected < 0.01,
				"NEMA {frame} ×{len}: want watertight×2 genus-0 spanning z −{len}…{}, ~{expected:.0}mm³; got {v:?} z=[{zmin:.1},{zmax:.1}] vol={vol:.0}",
				s.shaft_len
			);
		}
		assert!(nema_motor(8, 30.0).is_none() && nema_motor(17, 0.0).is_none(), "NEMA 8 and a zero-length body must be refused");
	}

	#[test]
	fn nema_mount_plate_carries_the_pilot_and_four_clearance_holes() {
		// N17 5 mm plate (margin 4) and N23 6 mm plate (margin 0): genus 5 = pilot +
		// 4 bolt holes, watertight on both routes, volume = slab − pilot bore − 4 ISO
		// 273 medium bores (1% band: hole tools are circumscribed 32/48-gons). The
		// pilot register bore is pilot_d + 0.2.
		for (frame, t, margin, clearance) in [(17usize, 5.0, 4.0, 3.4_f64), (23usize, 6.0, 0.0, 5.5)] {
			let s = nema_spec(frame).expect("frame row");
			let p = nema_mount_plate(frame, t, margin).expect("valid plate");
			let v = validate(&p);
			let side = s.face_w + 2.0 * margin;
			let expected = side * side * t - PI * ((s.pilot_d + 0.2) * 0.5).powi(2) * t - 4.0 * PI * (clearance * 0.5).powi(2) * t;
			let vol = volume(&p).abs();
			assert!(
				v.closed
					&& v.manifold && v.genus == 5
					&& tessellate_default(&p).is_watertight()
					&& tessellate_adaptive_tol(&p, 0.01).is_watertight()
					&& (vol - expected).abs() / expected < 0.01,
				"NEMA {frame} plate ×{t}: want watertight×2 genus-5 ~{expected:.0}mm³; got {v:?} vol={vol:.0}"
			);
		}
		assert!(nema_mount_plate(11, 5.0, 4.0).is_none() && nema_mount_plate(17, -1.0, 4.0).is_none(), "NEMA 11 and a negative thickness must be refused");
	}

	#[test]
	fn servo_pockets_cut_the_datasheet_case_and_screw_pattern() {
		// SG90 through a 4 mm panel (genus 3: pocket + 2 pilots) and MG996R through
		// 6 mm (genus 5: pocket + 4 pilots): watertight on both routes, volume =
		// panel − (case + 0.4 clearance) rect − pilot bores (1% band), and the pocket
		// walls sit at exactly ±(body_l/2 + 0.2) along x.
		for (name, t, holes, genus) in [("sg90", 4.0, 2.0, 3), ("mg996r", 6.0, 4.0, 5)] {
			let s = servo_spec(name).expect("servo row");
			let panel = cuboid(DVec3::new(-40.0, -20.0, 0.0), DVec3::new(40.0, 20.0, t));
			let cut = servo_pocket(&panel, DVec3::new(0.0, 0.0, t), DVec3::Z, name, t).expect("valid pocket");
			let v = validate(&cut);
			let expected = 80.0 * 40.0 * t - (s.body_l + 0.4) * (s.body_w + 0.4) * t - holes * PI * (s.pilot_d * 0.5).powi(2) * t;
			let vol = volume(&cut).abs();
			let wall_x = s.body_l * 0.5 + 0.2;
			let wall_verts = (0..cut.vertex_count() as u32)
				.map(|i| cut.position(VertexId(i)))
				.filter(|p| (p.x.abs() - wall_x).abs() < 1e-9)
				.count();
			assert!(
				v.closed
					&& v.manifold && v.genus == genus
					&& tessellate_default(&cut).is_watertight()
					&& tessellate_adaptive_tol(&cut, 0.01).is_watertight()
					&& wall_verts >= 8
					&& (vol - expected).abs() / expected < 0.01,
				"{name} pocket ×{t}: want watertight×2 genus-{genus} with walls at ±{wall_x}, ~{expected:.0}mm³; got {v:?} walls={wall_verts} vol={vol:.0}"
			);
		}
		let panel = cuboid(DVec3::new(-40.0, -20.0, 0.0), DVec3::new(40.0, 20.0, 4.0));
		assert!(
			servo_pocket(&panel, DVec3::new(0.0, 0.0, 4.0), DVec3::Z, "ds3218", 4.0).is_none()
				&& servo_pocket(&panel, DVec3::new(0.0, 0.0, 4.0), DVec3::ZERO, "sg90", 4.0).is_none(),
			"an unknown servo and a zero axis must be refused"
		);
	}
}
