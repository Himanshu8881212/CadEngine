// Copyright (c) LMCAD. Licensed under the MIT License.

//! **Heat-set threaded inserts** for 3D-printed parts: the boss-plus-pocket feature
//! that receives a brass insert. The insert itself is bought, not printed — what the
//! CAD model needs is the correctly *undersized* pocket (the insert melts its knurls
//! into the wall) and a boss with enough meat around it; both are produced here from
//! the published insert data.

use kernel_brep::geom::perp_basis;
use kernel_brep::math::{DAffine3, DMat3, DVec3};
use kernel_brep::{cylinder, difference, union, Solid};

/// One heat-set insert row: nominal thread, insert length, and the recommended
/// pilot-hole (pocket) diameter in the plastic.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HeatsetSpec {
	/// Nominal thread size (the "3" of M3).
	pub m: f64,
	/// Insert length (the pocket must be deeper — see [`heatset_insert_boss`]).
	pub length: f64,
	/// Recommended pilot/pocket diameter — *undersized* against the insert's
	/// knurled outside so the melt flows into the knurls.
	pub pilot_d: f64,
}

/// The heat-set insert table, M2–M6. Sources: the Ruthex insert range
/// (ruthex.de/en/pages/cad-daten — RX-M2x4, RX-M2.5x5.7, RX-M3x5.7, RX-M4x8.1,
/// RX-M5x9.5, RX-M6x12.7) and the matching Ruthex pilot drill set
/// (ruthex.de "HSS drill set for thread inserts": Ø 3.2 / 4.0 / 4.0 / 5.6 / 6.4 /
/// 8.0 mm for M2 / M2.5 / M3 / M4 / M5 / M6 — the M2.5 and M3 inserts share one
/// body). CNC Kitchen's M3 inserts take the same Ø 4.0 pilot.
static HEATSET_TABLE: [HeatsetSpec; 6] = [
	HeatsetSpec { m: 2.0, length: 4.0, pilot_d: 3.2 },
	HeatsetSpec { m: 2.5, length: 5.7, pilot_d: 4.0 },
	HeatsetSpec { m: 3.0, length: 5.7, pilot_d: 4.0 },
	HeatsetSpec { m: 4.0, length: 8.1, pilot_d: 5.6 },
	HeatsetSpec { m: 5.0, length: 9.5, pilot_d: 6.4 },
	HeatsetSpec { m: 6.0, length: 12.7, pilot_d: 8.0 },
];

/// All supported heat-set insert sizes (for table-driven callers).
pub fn heatset_specs() -> &'static [HeatsetSpec] {
	&HEATSET_TABLE
}

/// The insert row for nominal size `m` (2, 2.5, 3, 4, 5, 6), or `None`.
pub fn heatset_spec(m: f64) -> Option<&'static HeatsetSpec> {
	HEATSET_TABLE.iter().find(|s| (s.m - m).abs() < 1e-9)
}

/// Add a **heat-set insert boss** to a printed part: a cylindrical boss grown out of
/// the face at `at` along `axis` (the outward normal), with the correctly undersized
/// insert pocket bored back down its centre.
///
/// Sizing, from the cited table plus the common printed-boss rules of thumb:
/// - pocket Ø = the recommended pilot drill (e.g. M3 → Ø 4.0 — *not* the thread Ø);
/// - pocket depth = insert length + 1 mm (room for the melt pool below the insert);
/// - boss Ø = 2 × pilot Ø (≥ ~2 mm wall all round — e.g. the Ø 8 bosses Voron-style
///   printers use for M3 inserts);
/// - boss height = pocket depth + 2 mm floor.
///
/// The boss base must land fully on the host face (a contained coplanar contact —
/// the same proven fuse as a bolt head on its shank). Returns `None` for sizes
/// outside M2–M6 or a degenerate axis.
pub fn heatset_insert_boss(solid: &Solid, at: DVec3, axis: DVec3, m: f64) -> Option<Solid> {
	let spec = heatset_spec(m)?;
	let axis = axis.try_normalize()?;
	let pocket_depth = spec.length + 1.0;
	let boss_h = pocket_depth + 2.0;
	let (e1, e2) = perp_basis(axis);
	let frame = |origin: DVec3| DAffine3::from_mat3_translation(DMat3::from_cols(e1, e2, axis), origin);
	let boss = cylinder(DVec3::ZERO, DVec3::Z, spec.pilot_d, boss_h, 48).transformed(frame(at));
	let with_boss = union(solid, &boss);
	// The pocket plunges from 0.5 mm above the boss top down to the melt-pool floor.
	let pocket =
		cylinder(DVec3::new(0.0, 0.0, boss_h - pocket_depth), DVec3::Z, spec.pilot_d * 0.5, pocket_depth + 0.5, 48).transformed(frame(at));
	Some(difference(&with_boss, &pocket))
}

#[cfg(test)]
mod tests {
	use super::*;
	use kernel_brep::{cuboid, tessellate_default, validate, volume, VertexId};
	use std::f64::consts::PI;

	#[test]
	fn insert_bosses_grow_an_undersized_pocketed_boss_on_a_plate() {
		// M3 and M5 bosses on a 30×30×6 plate (boss up, +Z). The result must stay
		// genus 0 (blind pocket), be watertight, gain exactly boss − pocket volume
		// (48-gon faceting → 1%), and the pocket floor must sit 1 mm below the
		// insert: at z = 6 + 2 + 1 = 9 − … = plate + boss − (length + 1).
		for m in [3.0, 5.0] {
			let spec = heatset_spec(m).expect("table row");
			let plate = cuboid(DVec3::ZERO, DVec3::new(30.0, 30.0, 6.0));
			let bossed = heatset_insert_boss(&plate, DVec3::new(15.0, 15.0, 6.0), DVec3::Z, m).expect("table size");
			let v = validate(&bossed);
			let (depth, boss_h, boss_r) = (spec.length + 1.0, spec.length + 3.0, spec.pilot_d);
			let floor_z = 6.0 + boss_h - depth;
			let floor_verts = (0..bossed.vertex_count() as u32)
				.map(|i| bossed.position(VertexId(i)))
				.filter(|p| (p.z - floor_z).abs() < 1e-9 && ((p.x - 15.0).powi(2) + (p.y - 15.0).powi(2)).sqrt() < boss_r)
				.count();
			let expected = 30.0 * 30.0 * 6.0 + PI * boss_r * boss_r * boss_h - PI * (spec.pilot_d * 0.5).powi(2) * depth;
			assert!(
				v.closed
					&& v.manifold && v.genus == 0
					&& tessellate_default(&bossed).is_watertight()
					&& floor_verts > 0
					&& (volume(&bossed).abs() - expected).abs() / expected < 0.01,
				"M{m} insert boss: want watertight genus-0 with the pocket floor at z={floor_z}, ~{expected:.0}mm³; got {v:?} floor_verts={floor_verts} vol={:.0}",
				volume(&bossed).abs()
			);
		}
		let plate = cuboid(DVec3::ZERO, DVec3::new(30.0, 30.0, 6.0));
		assert!(
			heatset_insert_boss(&plate, DVec3::new(15.0, 15.0, 6.0), DVec3::Z, 8.0).is_none()
				&& heatset_insert_boss(&plate, DVec3::new(15.0, 15.0, 6.0), DVec3::ZERO, 3.0).is_none(),
			"M8 (out of table) and a zero axis must be refused"
		);
	}

	#[test]
	fn the_insert_table_is_the_published_ruthex_set() {
		// Snapshot of the cited table: designation lengths and pilot drills.
		let rows: Vec<(f64, f64, f64)> = heatset_specs().iter().map(|s| (s.m, s.length, s.pilot_d)).collect();
		assert_eq!(
			rows,
			vec![(2.0, 4.0, 3.2), (2.5, 5.7, 4.0), (3.0, 5.7, 4.0), (4.0, 8.1, 5.6), (5.0, 9.5, 6.4), (6.0, 12.7, 8.0),],
			"heat-set insert table must match the Ruthex RX-Mx and drill-set data"
		);
	}
}
