// Copyright (c) LMCAD. Licensed under the MIT License.

//! Standard-parts **catalog acceptance**: one of every part family in [`kernel_model::parts`],
//! each at two parameter sets, every body validated (closed/manifold/expected genus on the
//! exact B-rep, plus a watertight mesh via the exact-or-heal route) and written to
//! `catalog_out/` as STL. The fastener/key/washer sizes are the published standard rows
//! (ISO 4017 / ISO 4032 / ISO 7089 / DIN 912 / ISO 261 / DIN 6885-1 / GT2 2 mm / ANSI B29.1 —
//! see the `const` tables in `kernel-model/src/parts/`). Prints one table row per part and
//! exits non-zero if anything fails — this is the parts library's live acceptance gate.
//!
//! Run with: `cargo run --example catalog -p kernel-model --release`

use kernel_brep::math::DVec3;
use kernel_brep::{tessellate_adaptive_tol, tessellate_default, validate, volume, Solid};
use kernel_core::mesh::Mesh;
use kernel_model::parts::{
	as568_spec, board_mount_cut, bridged_counterbore, button_head_screw, chain_sprocket, circlip_external, circlip_groove_external,
	circlip_internal, clamp_coupling, compression_spring, deep_groove_bearing, din6885_key_size, dowel_pin, extrusion_2020,
	extrusion_3030,
	flanged_bearing, flat_head_screw, gear_rack, hose_barb,
	gt2_belt, gt2_center_distance, gt2_pulley, heatset_insert_boss, hex_bolt_iso4017, hex_nut_iso4032, internal_gear, iso286_fit,
	iso_thread_solid, jaw_coupling_hub, jaw_coupling_spider, kp08_pillow_block, lead_screw_nut_tr8, lead_screw_tr8,
	linear_bearing_lmuu, lock_nut, mgn12_carriage, mgn12_rail, nema_motor, nema_mount_plate, o_ring, o_ring_cord,
	o_ring_face_gland, o_ring_face_gland_racetrack, o_ring_groove, parallel_key, pc4_port_cut, pipe_boss_g, sc8uu_block,
	servo_pocket, set_screw,
	set_screw_coupling, shaft, shaft_support_shf8, shaft_support_sk8, shoulder_bolt, socket_head_cap_screw, spring_washer,
	spur_gear, standoff, teardrop_hole, threaded_hex_bolt, threaded_rod, thrust_bearing, tnut_2020, tr8_nut_trap,
	washer_iso7089, ShaftKeyway,
};
use kernel_model::{watertight_mesh, watertight_mesh_of};

const TOL: f64 = 0.01; // 10 µm chord tolerance for the exact tessellation path
const DIR: &str = "catalog_out";

/// Mesh a solid for display/printing, preferring the exact analytic routes: the adaptive
/// tessellation at [`TOL`], else the default exact tessellation (the adaptive stitcher can
/// crack on dense all-planar prisms where the default path is watertight), else healed
/// through the voxel half. Returns the route taken.
fn part_mesh(solid: &Solid) -> (Mesh, &'static str) {
	let adaptive = tessellate_adaptive_tol(solid, TOL);
	if adaptive.is_watertight() {
		return (adaptive, "exact-adaptive");
	}
	let default = tessellate_default(solid);
	if default.is_watertight() {
		return (default, "exact-default");
	}
	(watertight_mesh(solid, 0.3), "voxel-healed")
}

/// Validate one catalog entry, print its table row, and write `catalog_out/<file>.stl`.
/// Returns false if any structural check failed.
fn emit(file: &str, label: &str, solid: &Solid, want_genus: i64) -> bool {
	let v = validate(solid);
	let (mesh, route) = part_mesh(solid);
	let ok = v.closed && v.manifold && v.genus == want_genus && mesh.is_watertight();
	println!(
		"  {label:<34} closed={} manifold={} genus={} (want {want_genus})  vol={:>9.1} mm³  {:>6} tris via {:<12} watertight={}  {}",
		v.closed,
		v.manifold,
		v.genus,
		volume(solid).abs(),
		mesh.triangle_count(),
		route,
		mesh.is_watertight(),
		if ok { "PASS" } else { "FAIL" },
	);
	mesh.write_stl_binary(format!("{DIR}/{file}.stl")).expect("write stl");
	ok
}

/// Append `src` into `dst` as a triangle soup (no welding — the heal fuses them).
fn merge_into(dst: &mut Mesh, src: &Mesh) {
	let base = dst.positions.len() as u32;
	for p in &src.positions {
		dst.positions.push(*p);
	}
	for t in src.triangles() {
		dst.push_triangle(base + t[0], base + t[1], base + t[2]);
	}
}

fn main() {
	std::fs::create_dir_all(DIR).expect("create output dir");
	println!("Standard-parts catalog — two parameter sets per family:\n");
	let mut ok = true;

	// ISO 4017 hex bolts (genus 0: head stacked on shank).
	ok &= emit("hex_bolt_m10x30", "hex bolt ISO 4017 M10×30", &hex_bolt_iso4017(10.0, 30.0).expect("M10"), 0);
	ok &= emit("hex_bolt_m6x20", "hex bolt ISO 4017 M6×20", &hex_bolt_iso4017(6.0, 20.0).expect("M6"), 0);

	// ISO 4032 hex nuts (genus 1: one through-bore).
	ok &= emit("hex_nut_m10", "hex nut ISO 4032 M10", &hex_nut_iso4032(10.0).expect("M10"), 1);
	ok &= emit("hex_nut_m5", "hex nut ISO 4032 M5", &hex_nut_iso4032(5.0).expect("M5"), 1);

	// ISO 7089 plain washers (genus 1: annular rings) and DIN 127 B spring (split)
	// lock washers (genus 0: the split opens the ring into a helical strip).
	ok &= emit("washer_m10", "washer ISO 7089 M10", &washer_iso7089(10.0).expect("M10"), 1);
	ok &= emit("washer_m4", "washer ISO 7089 M4", &washer_iso7089(4.0).expect("M4"), 1);
	ok &= emit("spring_washer_m5", "spring washer DIN 127B M5", &spring_washer(5.0).expect("M5"), 0);
	ok &= emit("spring_washer_m8", "spring washer DIN 127B M8", &spring_washer(8.0).expect("M8"), 0);

	// ISO 7379 shoulder bolts (genus 0: thread stem + ground shoulder + socket head).
	ok &= emit("shoulder_bolt_8x20", "shoulder bolt ISO 7379 Ø8×20", &shoulder_bolt(8.0, 20.0).expect("Ø8"), 0);
	ok &= emit("shoulder_bolt_13x30", "shoulder bolt ISO 7379 Ø13×30", &shoulder_bolt(13.0, 30.0).expect("Ø13"), 0);

	// DIN 912 socket-head cap screws (genus 0: turned blank + hex socket pocket).
	ok &= emit("shcs_m6x20", "cap screw DIN 912 M6×20", &socket_head_cap_screw(6.0, 20.0).expect("M6"), 0);
	ok &= emit("shcs_m10x30", "cap screw DIN 912 M10×30", &socket_head_cap_screw(10.0, 30.0).expect("M10"), 0);

	// ISO 68-1 thread ridges (genus 0: a closed helical loft).
	ok &= emit("thread_m10", "thread ridge ISO M10×1.5 ×12", &iso_thread_solid(10.0, 1.5, 0.0, 12.0).expect("lofts"), 0);
	ok &= emit("thread_m6", "thread ridge ISO M6×1.0 ×10", &iso_thread_solid(6.0, 1.0, 0.0, 10.0).expect("lofts"), 0);

	// Involute spur gears, ISO 53 rack proportions (genus 1; first one DIN 6885-keyed).
	let key10 = din6885_key_size(10.0);
	ok &= emit("spur_gear_m2_z20", "spur gear m2 z20 keyed Ø10", &spur_gear(2.0, 20, 8.0, 10.0, 20.0, key10), 1);
	ok &= emit("spur_gear_m1p5_z48", "spur gear m1.5 z48 plain Ø8", &spur_gear(1.5, 48, 6.0, 8.0, 20.0, None), 1);

	// ISO 53 basic-rack gear racks (genus 0: all-planar bars, whole teeth only).
	ok &= emit("gear_rack_m2_x100", "gear rack m2 ×100 ×10 @20°", &gear_rack(2.0, 100.0, 10.0, 20.0).expect("rack"), 0);
	ok &= emit("gear_rack_m1_x30", "gear rack m1 ×30 ×6 @25°", &gear_rack(1.0, 30.0, 6.0, 25.0).expect("rack"), 0);

	// Internal ring gears (genus 1: the toothed bore through the rim).
	ok &= emit("internal_gear_m2_z36", "internal gear m2 z36 rim Ø84", &internal_gear(2.0, 36, 8.0, 84.0, 20.0).expect("ring"), 1);
	ok &= emit("internal_gear_m1p5_z24", "internal gear m1.5 z24 rim Ø44", &internal_gear(1.5, 24, 6.0, 44.0, 20.0).expect("ring"), 1);

	// GT2 2 mm timing pulleys (genus 1; one flanged, one plain).
	ok &= emit("gt2_pulley_z20_flanged", "GT2 pulley 20T ×6 flanged Ø5", &gt2_pulley(20, 6.0, 5.0, true), 1);
	ok &= emit("gt2_pulley_z16", "GT2 pulley 16T ×9 plain Ø5", &gt2_pulley(16, 9.0, 5.0, false), 1);

	// ANSI B29.1 chain sprockets (genus 1): #25 and #35 chains.
	ok &= emit("sprocket_25_z18", "sprocket ANSI #25 z18 Ø8", &chain_sprocket(6.35, 3.302, 18, 8.0), 1);
	ok &= emit("sprocket_35_z11", "sprocket ANSI #35 z11 Ø10", &chain_sprocket(9.525, 5.08, 11, 10.0), 1);

	// Shaft couplings: jaw hubs + spider (genus 1 each), set-screw rigid (genus 5:
	// bore + 4 set-screw tunnels), clamp (genus 4: slit-opened bore + 2 cross screws
	// through both lobes).
	ok &= emit("jaw_hub_d25_b8", "jaw coupling hub D25 Ø8", &jaw_coupling_hub(25.0, 8.0).expect("D25"), 1);
	ok &= emit("jaw_hub_d40_b12", "jaw coupling hub D40 Ø12", &jaw_coupling_hub(40.0, 12.0).expect("D40"), 1);
	ok &= emit("jaw_spider_d25", "jaw coupling spider D25", &jaw_coupling_spider(25.0).expect("D25"), 1);
	ok &= emit("setscrew_coupling_5x8", "set-screw coupling Ø5×Ø8", &set_screw_coupling(5.0, 8.0).expect("5×8"), 5);
	ok &= emit("clamp_coupling_8x10", "clamp coupling Ø8×Ø10", &clamp_coupling(8.0, 10.0).expect("8×10"), 4);

	// Linear motion: LM bearings (genus 1), SC8UU block (genus 1), SK8/SHF8 rod
	// supports (genus 4), MGN12 rail (genus = hole count) + carriage (genus 0).
	ok &= emit("lm8uu", "linear bearing LM8UU", &linear_bearing_lmuu(8.0).expect("LM8UU"), 1);
	ok &= emit("lm12uu", "linear bearing LM12UU", &linear_bearing_lmuu(12.0).expect("LM12UU"), 1);
	ok &= emit("sc8uu_block", "SC8UU bearing block", &sc8uu_block(), 1);
	ok &= emit("sk8_support", "SK8 shaft support", &shaft_support_sk8(), 4);
	ok &= emit("shf8_support", "SHF8 flange support", &shaft_support_shf8(), 4);
	ok &= emit("mgn12_rail_x200", "MGN12 rail ×200 (8 csk holes)", &mgn12_rail(200.0).expect("rail"), 8);
	ok &= emit("mgn12_carriage", "MGN12H carriage envelope", &mgn12_carriage(), 0);

	// Board mounting pattern: a base panel carrying a Raspberry Pi (cut from the
	// underside along +Z so the published corner-datum pattern lands literally).
	// Panel 105 × 80, NOT 105 × 76: on a 76-wide panel the hole rows sit exactly
	// 13.5 mm from BOTH y edges and that mirror symmetry cracks both tessellation
	// routes (valid B-rep, leaky stitch — pinned honestly in
	// parts::boards::tests::mirror_symmetric_panels_stay_valid_…, which proves the
	// voxel heal still delivers); the asymmetric panel stays on the exact route.
	let base = kernel_brep::cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(105.0, 80.0, 4.0));
	let pi_panel = board_mount_cut(&base, DVec3::new(10.0, 10.0, 0.0), DVec3::Z, "rpi").expect("rpi");
	ok &= emit("panel_rpi_mount", "panel + Raspberry Pi pattern", &pi_panel, 4);

	// Printing-native hole variants: a bracket wall with a Ø8 teardrop axle hole
	// (genus 1) and a plate whose bridged counterbore stays genus 0 — the
	// sacrificial membrane intentionally seals the bore until drilled out.
	let wall = kernel_brep::cuboid(DVec3::new(-10.0, -5.0, 0.0), DVec3::new(10.0, 5.0, 30.0));
	let axle = teardrop_hole(&wall, DVec3::new(0.0, 5.0, 15.0), -DVec3::Y, DVec3::Z, 8.0, 10.0).expect("teardrop");
	ok &= emit("wall_teardrop_8", "wall + Ø8 teardrop hole", &axle, 1);
	let cb_plate = kernel_brep::cuboid(DVec3::new(-15.0, -15.0, 0.0), DVec3::new(15.0, 15.0, 10.0));
	let bridged = bridged_counterbore(&cb_plate, DVec3::new(0.0, 0.0, 10.0), -DVec3::Z, 5.0, 10.0, 0.3).expect("bridge");
	ok &= emit("plate_bridged_cbore_m5", "plate + bridged cbore M5", &bridged, 0);

	// Fluid interfaces: G-port boss + hose barb (genus-1 revolves) and a manifold
	// plate with a PC4-M6 pneumatic port (genus 1: one tunnel).
	ok &= emit("pipe_boss_g14", "G1/4 port boss w2.5 ×12", &pipe_boss_g("G1/4", 2.5, 12.0).expect("G1/4"), 1);
	ok &= emit("pipe_boss_g12", "G1/2 port boss w3 ×15", &pipe_boss_g("G1/2", 3.0, 15.0).expect("G1/2"), 1);
	ok &= emit("hose_barb_6x3", "hose barb Ø6 ×3 teeth", &hose_barb(6.0, 3).expect("barb"), 1);
	let manifold = kernel_brep::cuboid(DVec3::new(-15.0, -15.0, 0.0), DVec3::new(15.0, 15.0, 10.0));
	let ported = pc4_port_cut(&manifold, DVec3::new(0.0, 0.0, 10.0), DVec3::Z, 6.0, 10.0).expect("PC4-M6");
	ok &= emit("manifold_pc4_m6", "manifold + PC4-M6 port", &ported, 1);

	// Bearing bodies (genus-1 revolves with ring-split witness grooves) and the
	// KP08 pillow block (genus 3: shaft bore + 2 bolt holes).
	ok &= emit("bearing_608", "608 bearing body 8×22×7", &deep_groove_bearing("608").expect("608"), 1);
	ok &= emit("bearing_6001", "6001 bearing body 12×28×8", &deep_groove_bearing("6001").expect("6001"), 1);
	ok &= emit("bearing_f623", "F623 flanged body 3×10×4", &flanged_bearing("F623").expect("F623"), 1);
	ok &= emit("bearing_51100", "51100 thrust body 10×24×9", &thrust_bearing("51100").expect("51100"), 1);
	ok &= emit("kp08_pillow_block", "KP08 pillow block", &kp08_pillow_block(), 3);

	// Tr8 lead-screw family (DIN 103): the Ø8 chamfered envelope (genus 0), the
	// flanged brass nut (genus 5), and a carriage plate with the nut trap (genus 5).
	ok &= emit("lead_screw_tr8x8_300", "lead screw Tr8×8 ×300", &lead_screw_tr8(300.0, 8.0).expect("Tr8×8"), 0);
	ok &= emit("lead_screw_tr8x2_100", "lead screw Tr8×2 ×100", &lead_screw_tr8(100.0, 2.0).expect("Tr8×2"), 0);
	ok &= emit("lead_screw_nut_tr8", "Tr8 flanged nut (brass envelope)", &lead_screw_nut_tr8(), 5);
	let carriage = kernel_brep::cuboid(DVec3::new(-25.0, -25.0, 0.0), DVec3::new(25.0, 25.0, 10.0));
	let trapped = tr8_nut_trap(&carriage, DVec3::new(0.0, 0.0, 10.0), DVec3::Z, 10.0).expect("trap");
	ok &= emit("carriage_tr8_nut_trap", "carriage + Tr8 nut trap", &trapped, 5);

	// NEMA motor interfaces: simplified bodies (genus 0), mount plates (genus 5:
	// pilot + 4 bolt holes), and servo-pocketed panels (genus 3/5).
	ok &= emit("nema17_motor_x40", "NEMA 17 motor body ×40", &nema_motor(17, 40.0).expect("N17"), 0);
	ok &= emit("nema23_motor_x56", "NEMA 23 motor body ×56", &nema_motor(23, 56.0).expect("N23"), 0);
	ok &= emit("nema17_plate_t5", "NEMA 17 mount plate ×5 +4", &nema_mount_plate(17, 5.0, 4.0).expect("N17"), 5);
	let panel = kernel_brep::cuboid(DVec3::new(-40.0, -20.0, 0.0), DVec3::new(40.0, 20.0, 4.0));
	let servo_panel = servo_pocket(&panel, DVec3::new(0.0, 0.0, 4.0), DVec3::Z, "sg90", 4.0).expect("SG90");
	ok &= emit("servo_panel_sg90", "panel + SG90 servo pocket", &servo_panel, 3);

	// Keyed shafts + their DIN 6885-1 parallel keys (genus 0).
	let kw20 = ShaftKeyway { size: din6885_key_size(20.0).expect("Ø20"), length: 25.0, offset: 10.0 };
	let kw12 = ShaftKeyway { size: din6885_key_size(12.0).expect("Ø12"), length: 16.0, offset: 8.0 };
	ok &= emit("shaft_d20_keyed", "shaft Ø20×60, DIN 6885 6×6×25", &shaft(20.0, 60.0, Some(kw20)), 0);
	ok &= emit("shaft_d12_keyed", "shaft Ø12×40, DIN 6885 4×4×16", &shaft(12.0, 40.0, Some(kw12)), 0);
	ok &= emit("key_6x6x25", "parallel key DIN 6885 A 6×6×25", &parallel_key(6.0, 6.0, 25.0), 0);
	ok &= emit("key_4x4x16", "parallel key DIN 6885 A 4×4×16", &parallel_key(4.0, 4.0, 16.0), 0);

	// ISO 10642 countersunk + ISO 7380 button-head socket screws (genus 0).
	ok &= emit("flat_head_m5x16", "flat head ISO 10642 M5×16", &flat_head_screw(5.0, 16.0).expect("M5"), 0);
	ok &= emit("flat_head_m10x30", "flat head ISO 10642 M10×30", &flat_head_screw(10.0, 30.0).expect("M10"), 0);
	ok &= emit("button_head_m5x16", "button head ISO 7380 M5×16", &button_head_screw(5.0, 16.0).expect("M5"), 0);
	ok &= emit("button_head_m8x20", "button head ISO 7380 M8×20", &button_head_screw(8.0, 20.0).expect("M8"), 0);

	// DIN 916 cup-point set screws (genus 0).
	ok &= emit("set_screw_m6x10", "set screw DIN 916 M6×10", &set_screw(6.0, 10.0).expect("M6"), 0);
	ok &= emit("set_screw_m10x16", "set screw DIN 916 M10×16", &set_screw(10.0, 16.0).expect("M10"), 0);

	// DIN 985 nyloc lock nuts (genus 1) — note the DIN 17/19 mm M10/M12 widths.
	ok &= emit("lock_nut_m10", "lock nut DIN 985 M10", &lock_nut(10.0).expect("M10"), 1);
	ok &= emit("lock_nut_m5", "lock nut DIN 985 M5", &lock_nut(5.0).expect("M5"), 1);

	// Threaded rod and hex standoffs (rod genus 0; bored standoff genus 1).
	ok &= emit("threaded_rod_m8x60", "threaded rod M8×60", &threaded_rod(8.0, 60.0).expect("M8"), 0);
	ok &= emit("standoff_m3x12", "hex standoff M3×12 AF5.5", &standoff(3.0, 12.0).expect("M3"), 1);

	// ISO 2338 dowel pins (genus 0: chamfered revolve).
	ok &= emit("dowel_pin_6x24", "dowel pin ISO 2338 Ø6×24", &dowel_pin(6.0, 24.0).expect("Ø6"), 0);
	ok &= emit("dowel_pin_3x12", "dowel pin ISO 2338 Ø3×12", &dowel_pin(3.0, 12.0).expect("Ø3"), 0);

	// DIN 471/472 circlips (genus 2: the two pliers holes) and a grooved shaft.
	ok &= emit("circlip_ext_20", "circlip DIN 471 Ø20 shaft", &circlip_external(20.0).expect("Ø20"), 2);
	ok &= emit("circlip_int_32", "circlip DIN 472 Ø32 bore", &circlip_internal(32.0).expect("Ø32"), 2);
	let plain20 = shaft(20.0, 40.0, None);
	let grooved =
		circlip_groove_external(&plain20, DVec3::new(0.0, 0.0, 32.0), DVec3::Z, 20.0).expect("Ø20 groove");
	ok &= emit("shaft_d20_circlip_groove", "shaft Ø20 + DIN 471 groove", &grooved, 0);

	// 2020/3030 extrusion stock + the matching M5 tee nut (all genus 1: one bore).
	ok &= emit("extrusion_2020_x100", "V-slot extrusion 2020 ×100", &extrusion_2020(100.0), 1);
	ok &= emit("extrusion_3030_x80", "T-slot extrusion 3030 ×80", &extrusion_3030(80.0), 1);
	ok &= emit("tnut_2020_m5", "drop-in tee nut 2020 M5", &tnut_2020(), 1);

	// Heat-set insert bosses grown on a printed plate (genus 0: blind pockets).
	let plate = kernel_brep::cuboid(DVec3::ZERO, DVec3::new(30.0, 30.0, 6.0));
	let bossed = heatset_insert_boss(&plate, DVec3::new(10.0, 15.0, 6.0), DVec3::Z, 3.0).expect("M3 boss");
	let bossed = heatset_insert_boss(&bossed, DVec3::new(22.0, 15.0, 6.0), DVec3::Z, 5.0).expect("M5 boss");
	ok &= emit("heatset_bosses_m3_m5", "heat-set bosses M3+M5 (Ruthex)", &bossed, 0);

	// AS568 O-rings (exact nominal tori, genus 1) and a Parker static gland turned
	// into the -214 ring's design shaft (Ø = ID + 2·L = 30.63).
	ok &= emit("o_ring_214", "O-ring AS568-214 24.99×3.53", &o_ring(214).expect("-214"), 1);
	ok &= emit("o_ring_012", "O-ring AS568-012 9.25×1.78", &o_ring(12).expect("-012"), 1);
	let g214 = as568_spec(214).expect("-214 row");
	let gland_shaft = kernel_brep::cylinder(DVec3::ZERO, DVec3::Z, g214.id * 0.5 + g214.gland_depth, 30.0, 48);
	let glanded = o_ring_groove(&gland_shaft, DVec3::new(0.0, 0.0, 12.0), DVec3::Z, 214).expect("-214 gland");
	ok &= emit("shaft_o_ring_gland_214", "shaft Ø30.63 + AS568-214 gland", &glanded, 0);

	// Metric cord rings (free ID — housing perimeters outrun the AS568 table) and the
	// matching face-seal (axial) glands: a circular gland on a round boss and the
	// racetrack lid gland of a rectangular housing (FRICTION #10).
	ok &= emit("o_ring_cord_150x3", "cord ring Ø150 × Ø3 metric", &o_ring_cord(150.0, 3.0).expect("stocked cord"), 1);
	ok &= emit("o_ring_cord_60x2", "cord ring Ø60 × Ø2 metric", &o_ring_cord(60.0, 2.0).expect("stocked cord"), 1);
	let boss = kernel_brep::cylinder(DVec3::ZERO, DVec3::Z, 25.0, 10.0, 48);
	let boss_gland = o_ring_face_gland(&boss, DVec3::new(0.0, 0.0, 10.0), DVec3::Z, 36.0, 2.0).expect("Ø2 face gland");
	ok &= emit("boss_face_gland_36x2", "boss Ø50 + face gland Ø36 (Ø2)", &boss_gland, 0);
	let lid = kernel_brep::cuboid(DVec3::new(-60.0, -40.0, 0.0), DVec3::new(60.0, 40.0, 6.0));
	let lid_gland =
		o_ring_face_gland_racetrack(&lid, DVec3::new(0.0, 0.0, 6.0), DVec3::Z, 100.0, 60.0, 8.0, 2.0).expect("racetrack gland");
	ok &= emit("lid_racetrack_gland", "lid 120×80 + racetrack 100×60 r8", &lid_gland, 0);

	// Compression springs (genus 0: a swept wire).
	ok &= emit("spring_d2_od16", "spring Ø2 wire, Ø16 ×5 turns", &compression_spring(2.0, 16.0, 6.0, 5.0).expect("spring"), 0);
	ok &= emit("spring_d1p5_od10", "spring Ø1.5 wire, Ø10 ×6.5 turns", &compression_spring(1.5, 10.0, 4.0, 6.5).expect("spring"), 0);

	// Threaded bolt showcase: the exact body+ridge pair fused through the voxel half (the
	// exact union self-intersects — the ridge pierces the shank wall — so the honest route to
	// ONE printable solid is the winding-number heal; see `threaded_hex_bolt`'s docs).
	let (body, thread) = threaded_hex_bolt(10.0, 30.0).expect("M10 in tables");
	let mut soup = tessellate_adaptive_tol(&body, TOL);
	merge_into(&mut soup, &tessellate_adaptive_tol(&thread, TOL));
	let fused = watertight_mesh_of(&soup, 0.2);
	let (vb, vf) = (volume(&body).abs(), fused.signed_volume().abs());
	let fused_ok = fused.is_watertight() && vf > vb;
	ok &= fused_ok;
	println!(
		"  {:<34} body+ridge fused via voxel half (0.2 mm): {} tris, watertight={}, vol {vf:.1} mm³ (> body {vb:.1})  {}",
		"threaded bolt M10×30 (fused)",
		fused.triangle_count(),
		fused.is_watertight(),
		if fused_ok { "PASS" } else { "FAIL" },
	);
	fused.write_stl_binary(format!("{DIR}/threaded_bolt_m10x30.stl")).expect("write stl");

	// Design-math utilities (no solids): GT2 belt sizing (the classic 20T/20T @ C100
	// → 240 mm / 120T identity, round-tripped through the inverse) and an ISO 286
	// preferred fit (Ø8 H7/g6 → published clearance 0.005–0.029 mm).
	let (belt_len, belt_teeth) = gt2_belt(100.0, 20, 20).expect("valid drive");
	let c = gt2_center_distance(belt_teeth, 20, 20).expect("belt fits");
	let fit = iso286_fit(8.0, "H7/g6").expect("supported fit");
	let math_ok = (belt_len - 240.0).abs() < 1e-9
		&& belt_teeth == 120
		&& (c - 100.0).abs() < 1e-9
		&& (fit.clearance.0 - 0.005).abs() < 1e-12
		&& (fit.clearance.1 - 0.029).abs() < 1e-12;
	ok &= math_ok;
	println!(
		"  {:<34} 20T/20T@C100 → {belt_len:.1} mm / {belt_teeth}T belt, inverse C={c:.3}; Ø8 H7/g6 clearance {:.3}–{:.3} mm  {}",
		"belt + fit design math",
		fit.clearance.0,
		fit.clearance.1,
		if math_ok { "PASS" } else { "FAIL" },
	);

	println!("\n{} — wrote STLs to ./{DIR}/", if ok { "ALL CATALOG PARTS PASS" } else { "SOME CATALOG PARTS FAILED" });
	std::process::exit(if ok { 0 } else { 1 });
}
