// Copyright (c) LMCAD. Licensed under the MIT License.

//! A McMaster-style library of ready-made **parametric standard parts**. Each part is a single
//! AI-callable function that returns an exact B-rep [`Solid`](kernel_brep::Solid) built from the
//! kernel's primitives, profile extrusions and booleans — so an agent can request a standard
//! component by dimension instead of reconstructing it every time.
//!
//! Conventions (used consistently across every function):
//! - dimensions are **millimetres**;
//! - bores, shanks and wires are given as **diameters**, never radii;
//! - hex sizes are **across flats** (the wrench size);
//! - standard dimensions are hardcoded `const` tables copied from the published standards, with
//!   the source cited next to each table — see [`fasteners`] (ISO 4017 / ISO 4032 / ISO 7089 /
//!   DIN 912), [`threads`] (ISO 261/262 coarse pitches, ISO 68-1 profile), [`shafts`]
//!   (DIN 6885-1 keys), [`pulleys`] (GT2 2 mm), [`sprockets`] (ANSI/ASA B29.1), [`pins`]
//!   (ISO 2338, DIN 471/472), [`screws`] (ISO 10642 / ISO 7380 / DIN 916 / DIN 985),
//!   [`inserts`] (Ruthex heat-set), [`extrusions`] (2020/3030 stock) and [`orings`]
//!   (AS568 + Parker ORD 5700 glands).
//!
//! Alongside the solids the library carries the matching pure **design math** — GT2 belt
//! sizing ([`gt2_belt`] / [`gt2_center_distance`]) and ISO 286 limit fits ([`iso286_fit`]) —
//! so an agent can pick a belt or a tolerance with the same cited-table rigor.
//!
//! Where a true form cannot be represented exactly (helical thread fusion, curvilinear GT2
//! flanks, trochoidal gear root fillets), the doc comment of the function says exactly what is
//! approximated and how — never silently.

mod bearings;
mod boards;
mod couplings;
mod extrusions;
mod fasteners;
mod fits;
mod fluid;
mod gears;
mod inserts;
mod leadscrews;
mod linear;
mod motors;
mod orings;
mod pins;
mod printing;
mod pulleys;
mod screws;
mod shafts;
mod springs;
mod sprockets;
mod threads;

pub use bearings::{
	deep_groove_bearing, flanged_bearing, flanged_bearing_spec, kp08_pillow_block, thrust_bearing, thrust_bearing_spec, FlangedBearingSpec,
	ThrustBearingSpec, SPLIT_GROOVE_DEPTH,
};
pub use boards::{board_mount_cut, board_pattern, BoardPattern};
pub use couplings::{
	clamp_coupling, clamp_coupling_spec, jaw_coupling_hub, jaw_coupling_spec, jaw_coupling_spider, set_screw_coupling,
	set_screw_coupling_spec, JawCouplingSpec, RigidCouplingSpec,
};
pub use extrusions::{extrusion_2020, extrusion_3030, tnut_2020};
pub use fasteners::{
	din127_dims, din912_dims, hex_bolt, hex_bolt_iso4017, hex_nut, hex_nut_iso4032, iso4017_head, socket_head_cap_screw, spring_washer,
	washer, washer_iso7089,
};
pub use fits::{iso286_fit, FitLimits};
pub use fluid::{g_thread_spec, hose_barb, pc4_port_cut, pipe_boss_g, GThreadSpec};
pub use gears::{
	cycloid_disc_profile, gear_rack, internal_gear, involute_ring_outline, involute_ring_outline_shifted,
	involute_ring_outline_shifted_filleted, involute_ring_outline_thinned, spur_gear, spur_gear_filleted, trapezoid_tooth_offsets,
};
pub use inserts::{heatset_insert_boss, heatset_spec, heatset_specs, HeatsetSpec};
pub use leadscrews::{lead_screw_nut_tr8, lead_screw_tr8, tr8_nut_trap, tr8_spec, tr8_thread_ridge, TrapezoidalSpec};
pub use linear::{
	linear_bearing_lmuu, lmuu_spec, mgn12_carriage, mgn12_rail, sc8uu_block, shaft_support_shf8, shaft_support_sk8, LmuuSpec,
};
pub use motors::{nema_motor, nema_mount_cut, nema_mount_plate, nema_spec, servo_pocket, servo_spec, NemaSpec, ServoSpec};
pub use orings::{
	as568_spec, metric_cord_gland, o_ring, o_ring_cord, o_ring_face_gland, o_ring_face_gland_racetrack, o_ring_groove,
	racetrack_cord_length, As568Spec, MetricCordGland,
};
pub use pins::{
	circlip_external, circlip_groove_external, circlip_groove_internal, circlip_internal, din471_spec, din472_spec, dowel_pin, CirclipSpec,
};
pub use printing::{bridged_counterbore, teardrop_hole};
pub use pulleys::{gt2_belt, gt2_center_distance, gt2_pulley};
pub use screws::{
	button_head_screw, din916_dims, din985_dims, flat_head_screw, iso10642_dims, iso7379_dims, iso7380_dims, lock_nut, set_screw,
	shoulder_bolt, standoff, threaded_rod,
};
pub use shafts::{din6885_key_size, parallel_key, shaft, KeySize, ShaftKeyway};
pub use springs::compression_spring;
pub use sprockets::chain_sprocket;
pub use threads::{iso_coarse_pitch, iso_thread_solid, threaded_hex_bolt};

use kernel_brep::geom::perp_basis;
use kernel_brep::math::{DAffine3, DMat3, DVec2, DVec3};
use kernel_brep::Solid;
use std::f64::consts::PI;

/// A 48-gon circle of radius `r` centred at the origin, starting at angle 0 and wound
/// counter-clockwise — the library's standard turning/boring resolution (the same ring the
/// `cylinder` primitive uses at 48 segments).
pub(crate) fn circle48(r: f64) -> Vec<DVec2> {
	(0..48)
		.map(|i| {
			let a = 2.0 * PI * i as f64 / 48.0;
			DVec2::new(r * a.cos(), r * a.sin())
		})
		.collect()
}

/// A regular hexagon of the given **across-flats** `width` (the wrench size = distance between two
/// opposite flats = 2 × apothem), centred at the origin in the XY plane with a flat parallel to X.
pub(crate) fn hexagon_across_flats(width: f64) -> Vec<DVec2> {
	// across-flats = 2·apothem and apothem = circumradius·cos30°, so circumradius = (w/2)/cos30°.
	let circumradius = (width * 0.5) / (PI / 6.0).cos();
	(0..6)
		.map(|i| {
			let a = PI / 6.0 + i as f64 * PI / 3.0;
			DVec2::new(circumradius * a.cos(), circumradius * a.sin())
		})
		.collect()
}

/// Area of a regular hexagon of the given **across-flats** width: `(√3/2)·w²` (test helper for
/// analytic volume expectations).
#[cfg(test)]
pub(crate) fn hexagon_area(width: f64) -> f64 {
	3.0_f64.sqrt() * 0.5 * width * width
}

/// Extrude `outer` to `height` along +Z and cut each listed hole through it with an
/// exact boolean **difference**, instead of handing the holes to
/// [`kernel_brep::extrude_with_holes`] as cap hole loops. The geometry is identical
/// (same polygons, bit-identical faceted volume) but the topology is not: boolean
/// face recovery rebuilds the caps as simple faces with NO inner loops — the one cap
/// kind the adaptive (chord-tolerance) tessellation stitcher can seam watertight —
/// so STL/3MF exports of these parts route `exact` instead of `voxel_healed`
/// (campaign/friction/ENGINE.md #6: the adaptive tessellator walks only outer loops, leaking at
/// every inner-loop cap). Every cutter overshoots both caps by 1 mm, so no cutter
/// face is coplanar with a cap (the proven transverse-cut route).
///
/// `circles` are `(centre, radius, segments)` round bores cut with the analytic
/// [`kernel_brep::cylinder`] primitive — at 48 segments its wall is the same 48-gon
/// as [`circle48`], but the wall faces carry the exact cylinder surface tag, so
/// `exact_volume` recovers the π-exact bore and STEP export writes a true cylinder
/// (campaign/friction/ENGINE.md #15). `prisms` are arbitrary CCW outlines (e.g. a keyway-notched
/// bore) cut as polygonal prisms. A degenerate hole (empty cutter) is skipped,
/// matching `extrude_with_holes`'s tolerance of degenerate hole loops.
pub(crate) fn extrude_bored(outer: &[DVec2], height: f64, circles: &[(DVec2, f64, usize)], prisms: &[Vec<DVec2>]) -> Solid {
	let mut solid = kernel_brep::extrude(outer, height);
	// Overshoot both caps regardless of extrusion sign (negative heights sweep down).
	let z0 = height.min(0.0) - 1.0;
	let h = height.abs() + 2.0;
	let cut = |solid: Solid, cutter: Solid| {
		if solid.face_count() > 0 && cutter.face_count() > 0 {
			kernel_brep::difference(&solid, &cutter)
		} else {
			solid
		}
	};
	for &(c, r, segments) in circles {
		solid = cut(solid, kernel_brep::cylinder(DVec3::new(c.x, c.y, z0), DVec3::Z, r, h, segments));
	}
	for hole in prisms {
		solid = cut(solid, kernel_brep::extrude(hole, h).transformed(DAffine3::from_translation(DVec3::new(0.0, 0.0, z0))));
	}
	solid
}

/// Cut an ISO 273 medium clearance hole **with a DIN 74-1 form-F 90° countersink**
/// as ONE plane-faceted loft cutter (32-gon frustum + bore cylinder, no analytic
/// cone tag) — the export-safe countersink: the analytic-cone cutter of
/// `kernel_brep::holes::countersink_hole` leaves a tagged conical face whose drilled
/// rim the adaptive tessellation stitcher cannot yet seam (the same kernel frontier
/// as the coupling cross-holes), which would push the part onto the voxel-heal STL
/// route — and cutting csk + bore as separate booleans proved fray-prone on faces
/// carrying many holes (the MGN rail), so the whole hole is a single difference.
/// Table data (clearance Ø, countersink Ø d2) comes from the same kernel
/// `metric_hole_spec` table; the cutter overshoots 1 mm beyond both faces. `at` on
/// the face, `axis` pointing INTO the material (the hole-wizard convention), the
/// bore through `through` mm of material. `None` outside the M3+ countersink table
/// or for a degenerate axis.
pub(crate) fn countersunk_hole_faceted(solid: &Solid, at: DVec3, axis: DVec3, m: f64, through: f64) -> Option<Solid> {
	let spec = kernel_brep::holes::metric_hole_spec(m)?;
	let csk_d = spec.countersink_d?;
	let axis = axis.try_normalize()?;
	if !(through > 0.0 && through.is_finite()) {
		return None;
	}
	let clearance_r = spec.clearance[1] * 0.5; // medium series
	let junction = csk_d * 0.5 - clearance_r; // 45° wall: radius shrinks 1:1 with depth
	let (e1, e2) = perp_basis(axis);
	// Depth coordinate: `axis` points INTO the material, so `at + axis·z` sinks
	// z below the face (z = −1 floats 1 mm above it).
	let ring = |r: f64, z: f64| -> Vec<DVec3> {
		(0..32)
			.map(|i| {
				let a = 2.0 * PI * i as f64 / 32.0;
				at + e1 * (r * a.cos()) + e2 * (r * a.sin()) + axis * z
			})
			.collect()
	};
	// ONE three-section loft cutter — 45° frustum from 1 mm proud of the face down
	// to the cone-bore junction, then the clearance cylinder overshooting 1 mm out
	// the far side — so each countersunk hole is a single boolean (per-hole cutter
	// pairs proved fray-prone on many-hole faces).
	let cutter = kernel_brep::loft_solid(&[ring(csk_d * 0.5 + 1.0, -1.0), ring(clearance_r, junction), ring(clearance_r, through + 1.0)])?;
	Some(kernel_brep::difference(solid, &cutter))
}

/// An annular **ring cutter** for lathe-style circumferential grooves: hole wall at
/// radius `r_hole` (the groove root), square outer boundary of apothem `r_clear + 2`
/// (radially clear of the Ø`2·r_clear` workpiece), `width` thick along the unit
/// `axis`, spanning `[at, at + width·axis]`. The square outer boundary — not a circle
/// — avoids the same-phase parallel-wall degeneracy of the cap-plane arrangement (see
/// `gt2_pulley`); the cutter's end caps cross the workpiece wall transversely.
pub(crate) fn ring_cutter(at: DVec3, axis: DVec3, r_hole: f64, r_clear: f64, width: f64) -> Solid {
	let h = r_clear + 2.0;
	let square = vec![DVec2::new(h, h), DVec2::new(-h, h), DVec2::new(-h, -h), DVec2::new(h, -h)];
	let (e1, e2) = perp_basis(axis);
	kernel_brep::extrude_with_holes(&square, &[circle48(r_hole)], width)
		.transformed(DAffine3::from_mat3_translation(DMat3::from_cols(e1, e2, axis), at))
}
