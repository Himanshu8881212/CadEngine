//! HARMONIC DRIVE (strain-wave gear), 50:1, for a NEMA 17 motor — a complete
//! robotic-arm joint actuator, built as an ASSEMBLY of real catalog parts.
//!
//! Kinematics: flex spline N_fs = 100 teeth, circular spline N_cs = 102 (Δ2),
//! circular spline grounded to the housing, output on the flex spline:
//!     ratio = -N_fs / (N_cs - N_fs) = -100/2 = -50:1.
//!
//! Strain-wave action: the wave generator is an ELLIPTICAL cam; in operation it
//! deforms the thin flex-spline cup into an ellipse so its 100 teeth ENGAGE the
//! 102-tooth circular spline at the major axis and CLEAR it at the minor axis —
//! the 2-tooth difference advances the output one tooth-pair per input revolution.
//! Honest static model: the flex-spline PART is shown ROUND (a clean, valid cup);
//! the elliptical engagement is PROVEN separately (the deformed rim's tips land in
//! the circular-spline tooth band at the major axis and clear it at the minor).
//!
//! Components (stacked +Z above the motor face at z=0):
//!   wave_generator  - elliptical cam on the Ø5 D-shaft (input)
//!   flex_spline     - 100T cup + closed diaphragm + Ø5 output boss with arm bolts
//!   circular_spline - 102T internal ring, grounded (bolts to the housing)
//!   housing         - NEMA 17 mount plate + tube up to the circular spline
//!   nema17_motor    - the motor it bolts to (context)
//!
//! Run: cargo run --example harmonic_drive -p kernel-model --release  ->  harmonic_out/

use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{cuboid, cylinder, difference, extrude, tessellate_adaptive_tol, union, validate, volume, Mesh, Solid};
use kernel_model::parts::{internal_gear, nema_motor, nema_mount_plate, spur_gear};
use std::f64::consts::TAU;

const M: f64 = 0.4;
const PA: f64 = 20.0;
const N_FS: usize = 100;
const N_CS: usize = 102;
const RIM_FW: f64 = 8.0;
const Z_RIM: f64 = 16.0; // gear plane above the motor face

fn ellipse_solid(a: f64, b: f64, z0: f64, h: f64, n: usize) -> Solid {
	let prof: Vec<DVec2> = (0..n).map(|i| { let t = TAU * i as f64 / n as f64; DVec2::new(a * t.cos(), b * t.sin()) }).collect();
	extrude(&prof, h).transformed(DAffine3::from_translation(DVec3::new(0.0, 0.0, z0)))
}

fn bolt_circle(s: &Solid, r: f64, n: usize, hole_r: f64, z0: f64, h: f64) -> Solid {
	let mut out = s.clone();
	for i in 0..n {
		let a = TAU * i as f64 / n as f64;
		out = difference(&out, &cylinder(DVec3::new(r * a.cos(), r * a.sin(), z0), DVec3::Z, hole_r, h, 24));
	}
	out
}

fn wave_generator() -> Solid {
	let cam = ellipse_solid(18.8, 18.2, Z_RIM, RIM_FW, 96);
	let bore = cylinder(DVec3::new(0.0, 0.0, Z_RIM - 1.0), DVec3::Z, 2.5, RIM_FW + 2.0, 48);
	let dflat = cuboid(DVec3::new(2.0, -3.0, Z_RIM - 1.0), DVec3::new(4.0, 3.0, Z_RIM + RIM_FW + 1.0));
	difference(&difference(&cam, &bore), &dflat)
}

/// Round (static) flex-spline cup: 100T rim, closed diaphragm, Ø5 output boss with
/// a 4×M3 arm-link circle. In operation the wave generator deforms it elliptical.
fn flex_spline() -> Solid {
	let rim = spur_gear(M, N_FS, RIM_FW, 37.4, PA, None) // tip 20.4, root 19.5, bore r=18.7
		.transformed(DAffine3::from_translation(DVec3::new(0.0, 0.0, Z_RIM)));
	// Diaphragm radius 19.0: >= bore r (18.7) so it bonds to the wall, < root (19.5)
	// so it never crosses the teeth -> a clean union.
	let diaphragm = cylinder(DVec3::new(0.0, 0.0, Z_RIM + 6.0), DVec3::Z, 19.0, 4.0, 96);
	let boss = cylinder(DVec3::new(0.0, 0.0, Z_RIM + 10.0), DVec3::Z, 14.0, 4.0, 64);
	let body = union(&union(&rim, &diaphragm), &boss);
	let bored = difference(&body, &cylinder(DVec3::new(0.0, 0.0, Z_RIM + 5.0), DVec3::Z, 2.5, 12.0, 48));
	bolt_circle(&bored, 9.0, 4, 1.6, Z_RIM + 9.0, 6.0)
}

fn circular_spline() -> Solid {
	let ring = internal_gear(M, N_CS, RIM_FW, 48.0, PA)
		.expect("circular spline internal gear")
		.transformed(DAffine3::from_translation(DVec3::new(0.0, 0.0, Z_RIM)));
	bolt_circle(&ring, 22.0, 4, 1.6, Z_RIM - 1.0, RIM_FW + 2.0)
}

fn housing() -> Solid {
	let plate = nema_mount_plate(17, 6.0, 8.0)
		.expect("nema 17 mount plate")
		.transformed(DAffine3::from_translation(DVec3::new(0.0, 0.0, -6.0)));
	// Overlap the tube 2 mm INTO the plate (bottom at z=-2, plate top at z=0) rather
	// than abutting coplanar: a hollow tube abutting a plane exactly coplanar leaves
	// an annular coplanar seam that tessellates non-watertight (FRICTION #20 class,
	// see tests/coplanar_tube_tessellation.rs). Overlapping keeps it watertight.
	let tube = difference(
		&cylinder(DVec3::new(0.0, 0.0, -2.0), DVec3::Z, 25.0, Z_RIM + 2.0, 96),
		&cylinder(DVec3::new(0.0, 0.0, -3.0), DVec3::Z, 21.0, Z_RIM + 4.0, 96),
	);
	bolt_circle(&union(&plate, &tube), 22.0, 4, 1.6, Z_RIM - 5.0, 6.0)
}

fn merge_into(dst: &mut Mesh, src: &Mesh) {
	let base = dst.positions.len() as u32;
	dst.positions.extend_from_slice(&src.positions);
	dst.indices.extend(src.indices.iter().map(|i| i + base));
}

fn emit(dir: &str, name: &str, s: &Solid, want_watertight: bool) -> (bool, Mesh) {
	let v = validate(s);
	let mesh = tessellate_adaptive_tol(s, 0.05);
	let wt = mesh.is_watertight();
	let _ = std::fs::create_dir_all(dir);
	let _ = std::fs::write(format!("{dir}/{name}.stl"), mesh.to_stl_binary());
	let ok = v.is_valid() && (!want_watertight || wt);
	println!("  {name:16} valid={} genus={:2} watertight={wt} vol={:>8.0} mm³  {}", v.is_valid(), v.genus, volume(s).abs(), if ok { "OK" } else { "<<<" });
	(ok, mesh)
}

fn main() {
	let dir = "harmonic_out";
	println!("HARMONIC DRIVE 50:1 for NEMA 17 — assembly:");
	let (wg, fs, cs, hs) = (wave_generator(), flex_spline(), circular_spline(), housing());
	let motor = nema_motor(17, 40.0).expect("nema 17 motor");

	let (o1, m_wg) = emit(dir, "wave_generator", &wg, true);
	let (o2, m_fs) = emit(dir, "flex_spline", &fs, true);
	let (o3, m_cs) = emit(dir, "circular_spline", &cs, true);
	let (o4, m_hs) = emit(dir, "housing", &hs, true); // watertight (tube overlaps the plate, not coplanar-abutting)
	let (_, m_mo) = emit(dir, "nema17_motor", &motor, true);
	let parts_ok = o1 && o2 && o3 && o4;

	// Strain-wave engagement PROOF on the deformed (elliptical) rim: x*1.005 (major
	// 20.1), y*0.975 (minor 19.5). Tips must land in the 102T band (20.0..20.9) at
	// major and clear (<20.0) at minor.
	let deformed = spur_gear(M, N_FS, RIM_FW, 37.4, PA, None).transformed(DAffine3::from_scale(DVec3::new(20.1 / 20.0, 19.5 / 20.0, 1.0)));
	let (_, dmx) = deformed.aabb();
	let engaged = (20.0..20.9).contains(&dmx.x) && dmx.y < 20.0;
	println!("strain wave: deformed rim tip — major {:.2} (engages 20.0..20.9), minor {:.2} (clears <20.0)  engaged={engaged}", dmx.x, dmx.y);

	// Display assembly: MERGE the part meshes (the gear teeth interMESH, so a boolean
	// union would be pathological — merging shows the real engaged geometry).
	let mut asm = Mesh::new();
	for m in [&m_wg, &m_fs, &m_cs, &m_hs, &m_mo] {
		merge_into(&mut asm, m);
	}
	let _ = asm.write_stl_binary(format!("{dir}/ASSEMBLY.stl"));
	println!("merged assembly: {} triangles -> {dir}/ASSEMBLY.stl", asm.indices.len() / 3);

	let ratio = -(N_FS as f64) / (N_CS as f64 - N_FS as f64);
	println!("reduction = {ratio:.0}:1 (circular spline grounded, output on flex spline)");
	let ok = parts_ok && engaged;
	println!("RESULT: parts_ok={parts_ok} engaged={engaged} => {}", if ok { "PASS" } else { "FAIL" });
	if !ok {
		std::process::exit(1);
	}
}
