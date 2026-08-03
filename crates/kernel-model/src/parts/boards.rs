// Copyright (c) LMCAD. Licensed under the MIT License.

//! **Board mounting patterns**: one-call clearance-hole patterns for the boards
//! every enclosure ends up carrying — Raspberry Pi, Arduino Uno, and the VESA
//! FDMI 75/100 squares (NUCs, monitors, mini-PCs). Each pattern cuts the proper
//! ISO 273 medium clearance holes through everything along the axis, on the
//! pattern's **own published datum** (no re-derived numbers):
//!
//! - `"rpi"` — vendor drawing origin = board bottom-left corner;
//! - `"arduino_uno"` — reference drawing origin = board bottom-left corner;
//! - `"vesa75"` / `"vesa100"` — the FDMI measures from the **pattern centre**.

use super::perp_basis;
use kernel_brep::holes::{clearance_hole, Fit};
use kernel_brep::math::DVec3;
use kernel_brep::Solid;

/// One board-pattern row: designation, screw size, hole positions on the
/// pattern's published datum, and the board outline (span relative to the same
/// datum) for sizing a pocket or standoff field around it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BoardPattern {
	/// Designation, e.g. `"rpi"`.
	pub designation: &'static str,
	/// Mounting screw nominal size (M2.5, M3, M4 — ISO 273 medium holes).
	pub m: f64,
	/// Hole centres `(x, y)` in the face frame, mm, on the published datum.
	pub holes: &'static [(f64, f64)],
	/// Board outline `(x_min, y_min, x_max, y_max)` around the same datum.
	pub outline: (f64, f64, f64, f64),
}

/// Raspberry Pi B-family holes: 4 × M2.5 at (3.5, 3.5) + 58 × 49 on the 85 × 56
/// board. Source: the Raspberry Pi mechanical drawings (raspberrypi.com
/// documentation) — the pattern is shared by B+/2B/3B/4B/5.
const RPI_HOLES: [(f64, f64); 4] = [(3.5, 3.5), (61.5, 3.5), (3.5, 52.5), (61.5, 52.5)];

/// Arduino Uno R3 holes: 4 × Ø3.2 (M3) at the reference-drawing positions
/// (inch-grid coordinates × 25.4) on the 68.58 × 53.34 board. Source: the
/// Arduino Uno mechanical/EAGLE reference drawing.
const UNO_HOLES: [(f64, f64); 4] = [(13.97, 2.54), (66.04, 7.62), (66.04, 35.56), (15.24, 50.8)];

/// VESA FDMI MIS-D 75: 75 × 75 M4 square about the pattern centre.
const VESA75_HOLES: [(f64, f64); 4] = [(-37.5, -37.5), (37.5, -37.5), (-37.5, 37.5), (37.5, 37.5)];

/// VESA FDMI MIS-D 100: 100 × 100 M4 square about the pattern centre.
const VESA100_HOLES: [(f64, f64); 4] = [(-50.0, -50.0), (50.0, -50.0), (-50.0, 50.0), (50.0, 50.0)];

/// The supported board patterns (see the module doc for each datum).
const BOARDS: [BoardPattern; 4] = [
	BoardPattern { designation: "rpi", m: 2.5, holes: &RPI_HOLES, outline: (0.0, 0.0, 85.0, 56.0) },
	BoardPattern { designation: "arduino_uno", m: 3.0, holes: &UNO_HOLES, outline: (0.0, 0.0, 68.58, 53.34) },
	BoardPattern { designation: "vesa75", m: 4.0, holes: &VESA75_HOLES, outline: (-37.5, -37.5, 37.5, 37.5) },
	BoardPattern { designation: "vesa100", m: 4.0, holes: &VESA100_HOLES, outline: (-50.0, -50.0, 50.0, 50.0) },
];

/// The pattern row for `"rpi"`, `"arduino_uno"`, `"vesa75"` or `"vesa100"`
/// (case-insensitive), or `None`.
pub fn board_pattern(designation: &str) -> Option<BoardPattern> {
	BOARDS.iter().find(|b| b.designation.eq_ignore_ascii_case(designation)).copied()
}

/// Cut a **board mounting pattern** into a panel: the ISO 273 medium clearance
/// holes (M2.5 for `"rpi"`, M3 for `"arduino_uno"`, M4 for `"vesa75"`/
/// `"vesa100"`) through ALL material along `axis`, at the published positions.
/// `at` is the pattern datum on the face — the board's bottom-left corner for
/// rpi/arduino (the vendors' drawing origin), the pattern centre for VESA —
/// and `axis` points INTO the material (the hole-wizard convention); hole x/y
/// run along the face frame of `axis` (`perp_basis`: for +Z that is exactly
/// world (X, Y); for −Z it is (X, −Y), i.e. a top-face pattern mirrors in y —
/// the corner-anchored rpi/arduino patterns are chiral, so check which way `y`
/// runs, or cut from the underside with +Z). Adds one tunnel per hole
/// (genus +4 on a plate). Mount the board on
/// [`super::standoff`]s or [`super::heatset_insert_boss`]es at the same
/// positions. `None` for an unknown designation or degenerate axis.
pub fn board_mount_cut(solid: &Solid, at: DVec3, axis: DVec3, designation: &str) -> Option<Solid> {
	let board = board_pattern(designation)?;
	let axis = axis.try_normalize()?;
	let (e1, e2) = perp_basis(axis);
	let mut s = solid.clone();
	for &(x, y) in board.holes {
		s = clearance_hole(&s, at + e1 * x + e2 * y, axis, board.m, Fit::Medium, None).ok()?;
	}
	Some(s)
}

#[cfg(test)]
mod tests {
	use super::*;
	use kernel_brep::{cuboid, tessellate_adaptive_tol, tessellate_default, validate, volume, VertexId};
	use std::f64::consts::PI;

	#[test]
	fn board_patterns_cut_their_published_holes_through_the_panel() {
		// All four patterns into a 130 × 130 × 4 panel, cut from the underside along
		// +Z so the face frame is exactly world (X, Y) and the published positions
		// land literally: genus 4, watertight on both routes, every hole position
		// carries a wall-vertex ring of its ISO 273 medium radius, and the volume
		// drops by exactly 4 clearance cylinders (1% for hole faceting). An unknown
		// board is refused.
		let panel = cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(130.0, 130.0, 4.0));
		let mut all = true;
		let mut diag = String::new();
		for (des, datum) in [("rpi", (20.0, 20.0)), ("arduino_uno", (20.0, 20.0)), ("vesa75", (65.0, 65.0)), ("vesa100", (65.0, 65.0))] {
			let b = board_pattern(des).expect("table row");
			let at = DVec3::new(datum.0, datum.1, 0.0);
			let cut = board_mount_cut(&panel, at, DVec3::Z, des).expect("pattern fits the panel");
			let v = validate(&cut);
			let wt = tessellate_default(&cut).is_watertight() && tessellate_adaptive_tol(&cut, 0.01).is_watertight();
			let r = kernel_brep::holes::metric_hole_spec(b.m).expect("hole row").clearance[1] * 0.5;
			let rings_ok = b.holes.iter().all(|&(x, y)| {
				let c = at + DVec3::new(x, y, 0.0);
				(0..cut.vertex_count() as u32)
					.map(|i| cut.position(VertexId(i)))
					.filter(|p| (((p.x - c.x).powi(2) + (p.y - c.y).powi(2)).sqrt() - r).abs() < 1e-9)
					.count() >= 32
			});
			let expected = 130.0 * 130.0 * 4.0 - 4.0 * PI * r * r * 4.0;
			let vol = volume(&cut).abs();
			let ok = v.closed && v.manifold && v.genus == 4 && wt && rings_ok && (vol - expected).abs() / expected < 0.01;
			if !ok {
				diag += &format!("{des}: {v:?} wt={wt} rings={rings_ok} vol={vol:.1} want={expected:.1}; ");
			}
			all &= ok;
		}
		assert!(
			all && board_mount_cut(&panel, DVec3::new(10.0, 10.0, 4.0), -DVec3::Z, "rpi_pico").is_none(),
			"every board pattern must cut genus-4 watertight×2 holes at its published positions (and rpi_pico is not stocked); failures: {diag}"
		);
	}

	#[test]
	fn top_face_patterns_mirror_in_y_as_documented() {
		// The documented chirality caveat is real, not folklore: the same rpi cut
		// from the TOP face (axis −Z, frame (X, −Y)) puts the (3.5, 52.5) hole at
		// world y = datum − 52.5. One witness ring proves it.
		let panel = cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(130.0, 130.0, 4.0));
		let cut = board_mount_cut(&panel, DVec3::new(20.0, 80.0, 4.0), -DVec3::Z, "rpi").expect("fits");
		let r = 2.9 * 0.5; // ISO 273 medium for M2.5
		let c = DVec3::new(20.0 + 3.5, 80.0 - 52.5, 0.0);
		let ring = (0..cut.vertex_count() as u32)
			.map(|i| cut.position(VertexId(i)))
			.filter(|p| (((p.x - c.x).powi(2) + (p.y - c.y).powi(2)).sqrt() - r).abs() < 1e-9)
			.count();
		assert!(
			validate(&cut).genus == 4 && ring >= 32,
			"a −Z rpi pattern must land its (3.5, 52.5) hole at y = datum − 52.5; ring verts there: {ring}"
		);
	}

	#[test]
	fn mirror_symmetric_panels_stay_valid_and_route_watertight_via_the_heal() {
		// KNOWN KERNEL FRONTIER, pinned honestly: when the two rpi hole rows land
		// exactly equidistant from the panel's y edges (here 13.5 mm both sides on a
		// 76-wide panel), the B-rep is perfectly VALID (closed, manifold, genus 4)
		// but BOTH tessellation routes crack — a stitcher degeneracy on
		// mirror-symmetric hole arrangements (a 1.5 mm datum shift heals it; see the
		// passing asymmetric cases in the test above). This test asserts what IS
		// true: validity holds, and the hybrid `watertight_mesh` router still
		// delivers a watertight mesh (the voxel heal — the documented honest route).
		// If the stitcher is ever fixed, tessellate_default(&cut).is_watertight()
		// will flip true here and this test should be tightened to the exact route.
		let panel = cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(105.0, 76.0, 4.0));
		let cut = board_mount_cut(&panel, DVec3::new(10.0, 10.0, 0.0), DVec3::Z, "rpi").expect("fits");
		let v = validate(&cut);
		let healed = crate::watertight_mesh(&cut, 0.3);
		assert!(
			v.closed && v.manifold && v.genus == 4 && healed.is_watertight(),
			"the symmetric-panel rpi cut must stay a valid genus-4 B-rep and route watertight via the voxel heal; got {v:?} healed_wt={}",
			healed.is_watertight()
		);
	}

	#[test]
	fn pattern_tables_match_the_vendor_drawings() {
		// The cited numbers themselves: Pi 58 × 49 from (3.5, 3.5) M2.5 on an 85 × 56
		// board; Uno's four inch-grid positions M3; VESA 75/100 M4 squares about the
		// centre. One snapshot over the spec rows.
		let rows: Vec<_> = ["rpi", "arduino_uno", "vesa75", "vesa100", "nuc"]
			.iter()
			.map(|d| board_pattern(d).map(|b| (b.m, b.holes.to_vec(), b.outline)))
			.collect();
		assert_eq!(
			rows,
			vec![
				Some((2.5, vec![(3.5, 3.5), (61.5, 3.5), (3.5, 52.5), (61.5, 52.5)], (0.0, 0.0, 85.0, 56.0))),
				Some((3.0, vec![(13.97, 2.54), (66.04, 7.62), (66.04, 35.56), (15.24, 50.8)], (0.0, 0.0, 68.58, 53.34))),
				Some((4.0, vec![(-37.5, -37.5), (37.5, -37.5), (-37.5, 37.5), (37.5, 37.5)], (-37.5, -37.5, 37.5, 37.5))),
				Some((4.0, vec![(-50.0, -50.0), (50.0, -50.0), (-50.0, 50.0), (50.0, 50.0)], (-50.0, -50.0, 50.0, 50.0))),
				None
			],
			"board-pattern table rows (a NUC mounts via the VESA rows; 'nuc' itself is not a designation)"
		);
	}
}
