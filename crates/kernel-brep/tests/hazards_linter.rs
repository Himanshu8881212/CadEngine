//! The boolean hazard linter must NAME the three input patterns that sit in
//! the arrangement's least-margin corner (all three bit the RESPOOL campaign,
//! 2026-07-28): exact coincident faces (supported, informational),
//! nearly-coincident faces (the sliver band), and a straight edge of one
//! operand lying inside a planar face of the other (the facet-meridian /
//! coplanar-overlap-edge class) — and must stay QUIET for well-separated
//! operands.

use kernel_brep::math::{DVec2, DVec3};
use kernel_brep::{boolean_hazards, cuboid, revolve, sector_prism, HazardKind};

fn tube(seg: usize) -> kernel_brep::Solid {
	revolve(&[DVec2::new(37.3, 0.0), DVec2::new(40.5, 0.0), DVec2::new(40.5, 12.0), DVec2::new(37.3, 12.0)], seg)
}

#[test]
fn linter_names_all_three_hazard_classes_and_stays_quiet_when_clean() {
	// (1) EXACT flush stack: boss on plate — the supported cancel path, reported
	// as informational CoincidentPlanes.
	let plate = cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(30.0, 20.0, 10.0));
	let boss_flush = cuboid(DVec3::new(5.0, 5.0, 10.0), DVec3::new(25.0, 15.0, 18.0));
	let flush = boolean_hazards(&plate, &boss_flush, 0.05);
	let flush_hit = flush.iter().any(|h| h.kind == HazardKind::CoincidentPlanes && h.separation <= 1e-7);

	// (2) NEAR-coincident: the same boss floated 0.02 above — the dangerous
	// sliver band the linter exists for.
	let boss_sliver = cuboid(DVec3::new(5.0, 5.0, 10.02), DVec3::new(25.0, 15.0, 18.0));
	let sliver = boolean_hazards(&plate, &boss_sliver, 0.05);
	let sliver_hit = sliver.iter().any(|h| h.kind == HazardKind::NearCoincidentPlanes && (h.separation - 0.02).abs() < 1e-6);

	// (3) EdgeInFace: a sector cutter whose side plane lies exactly ON a facet
	// meridian of a SEG=120 revolve (pitch 3°; 171° = facet boundary 57) — the
	// meridian edges lie in the cutter's plane, inside its face region. The
	// same cutter against a SEG=126 tube (pitch 2.857°; nearest meridians
	// 168.57° / 171.43°) is ~0.9 mm clear of every meridian and must NOT fire.
	let cutter = sector_prism(30.0, 45.0, 171.0, 249.0, 4.0, 14.0, 2.0);
	let on_grid = boolean_hazards(&tube(120), &cutter, 0.05);
	let on_grid_hit = on_grid.iter().any(|h| h.kind == HazardKind::EdgeInFace);
	let off_grid = boolean_hazards(&tube(126), &cutter, 0.05);
	let off_grid_edge_hits = off_grid.iter().filter(|h| h.kind == HazardKind::EdgeInFace).count();

	// (4) quiet when clean: the boss floated a full 1.0 above the plate.
	let boss_clear = cuboid(DVec3::new(5.0, 5.0, 11.0), DVec3::new(25.0, 15.0, 18.0));
	let clear = boolean_hazards(&plate, &boss_clear, 0.05);

	assert!(
		flush_hit && sliver_hit && on_grid_hit && off_grid_edge_hits == 0 && clear.is_empty(),
		"hazard linter: flush stack CoincidentPlanes={flush_hit} (want true); 0.02 sliver NearCoincidentPlanes={sliver_hit} \
		 (want true); SEG=120 on-meridian cutter EdgeInFace={on_grid_hit} (want true); SEG=126 off-meridian EdgeInFace \
		 count={off_grid_edge_hits} (want 0); clean 1.0-gap report len={} (want 0). Full reports:\nflush: {:?}\nsliver: {:?}\non_grid: {:?}",
		clear.len(),
		flush,
		sliver,
		on_grid
	);
}
