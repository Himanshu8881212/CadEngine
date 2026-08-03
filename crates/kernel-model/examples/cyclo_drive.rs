//! CYCLOIDAL DRIVE 10:1 — a modular robot-joint actuator for a NEMA 17, built as
//! a HYBRID assembly: the gear train is exact B-REP (cycloidal disc, pin ring,
//! eccentric cam, output flange) and the link ARM is an IMPLICIT GYROID LATTICE
//! (max strength-to-weight). The arm's far end is an ORTHOGONAL NEMA-17 mount face
//! so the next motor+drive bolts on perpendicular — chain these for a multi-axis arm.
//!
//! Kinematics: cycloidal disc with Zc = 10 lobes rolling inside a ring of
//! Zp = 11 pins, ring grounded, output via roller pins through the disc's output
//! holes => reduction = Zc/(Zp-Zc) = 10:1. The eccentric cam (offset E) on the
//! Ø5 shaft wobbles the disc; each input turn advances it one pin.
//!
//! Run: cargo run --example cyclo_drive -p kernel-model --release  ->  cyclo_out/

use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{cylinder, difference, extrude, tessellate_default, union, validate, volume, Mesh, Solid};
use kernel_core::math::{Aabb, Vec3};
use kernel_implicit::{dual_contour_narrowband, Cuboid, Node, Resolution, Tpms, TpmsKind};
use kernel_model::parts::nema_mount_plate;
use std::f64::consts::TAU;

const ZP: f64 = 11.0; // ring pins  (lobes = 10 -> 10:1)
const RP: f64 = 20.0; // pin circle radius
const RR: f64 = 2.0; // pin radius
const E: f64 = 1.2; // eccentricity
const DT: f64 = 6.0; // disc thickness
const Z0: f64 = 8.0; // disc plane (above the motor face at z=0)
const ARM_LEN: f64 = 80.0;

fn cycloid_profile(n: usize) -> Vec<DVec2> {
	let p: Vec<DVec2> = (0..n)
		.map(|i| {
			let t = TAU * i as f64 / n as f64;
			let psi = ((1.0 - ZP) * t).sin().atan2(RP / (E * ZP) - ((1.0 - ZP) * t).cos());
			DVec2::new(
				RP * t.cos() - RR * (t + psi).cos() - E * (ZP * t).cos(),
				-RP * t.sin() + RR * (t + psi).sin() + E * (ZP * t).sin(),
			)
		})
		.collect();
	// Cycloidal equations wind clockwise; extrude wants CCW.
	let area: f64 = 0.5 * (0..n).map(|i| { let j = (i + 1) % n; p[i].x * p[j].y - p[j].x * p[i].y }).sum::<f64>();
	if area < 0.0 { p.into_iter().rev().collect() } else { p }
}

/// Cycloidal disc, placed at the eccentric pose (centre offset +E): central
/// bearing bore, 10 lobes, 6 output-pin holes (Ø8, so the disc wobbles E around
/// Ø5.6 output pins).
fn cycloid_disc() -> Solid {
	let disc = extrude(&cycloid_profile(360), DT)
		.transformed(DAffine3::from_translation(DVec3::new(E, 0.0, Z0)));
	let mut d = difference(&disc, &cylinder(DVec3::new(E, 0.0, Z0 - 1.0), DVec3::Z, 6.0, DT + 2.0, 48));
	for i in 0..6 {
		let a = TAU * i as f64 / 6.0;
		d = difference(&d, &cylinder(DVec3::new(E + 12.0 * a.cos(), 12.0 * a.sin(), Z0 - 1.0), DVec3::Z, 4.0, DT + 2.0, 32));
	}
	d
}

/// Pin-ring housing: an OD50 ring carrying 11 inward pins on the Ø40 circle, on a
/// NEMA-17 mount plate. Grounded.
fn pin_ring_housing() -> Solid {
	// One continuous shell tube (no internal coplanar tube abutments), bore r=21.
	let mut h = difference(
		&cylinder(DVec3::new(0.0, 0.0, -2.0), DVec3::Z, 25.0, Z0 + DT + 4.0, 96),
		&cylinder(DVec3::new(0.0, 0.0, -3.0), DVec3::Z, 21.0, Z0 + DT + 6.0, 96),
	);
	for i in 0..(ZP as usize) {
		let a = TAU * i as f64 / ZP;
		h = union(&h, &cylinder(DVec3::new(RP * a.cos(), RP * a.sin(), Z0 - 1.0), DVec3::Z, RR, DT + 2.0, 24));
	}
	// NEMA 17 mount plate (z -6..0) OVERLAPS the shell base (shell from z=-2).
	let plate = nema_mount_plate(17, 6.0, 8.0).expect("nema plate").transformed(DAffine3::from_translation(DVec3::new(0.0, 0.0, -6.0)));
	union(&h, &plate)
}

/// Eccentric cam on the Ø5 D-shaft: a cylinder (the bearing journal) offset by E.
fn eccentric_cam() -> Solid {
	let cam = cylinder(DVec3::new(E, 0.0, Z0 - 1.0), DVec3::Z, 5.8, DT + 2.0, 64);
	let bore = cylinder(DVec3::new(0.0, 0.0, Z0 - 2.0), DVec3::Z, 2.5, DT + 4.0, 48);
	let dflat = kernel_brep::cuboid(DVec3::new(2.0, -3.0, Z0 - 2.0), DVec3::new(4.0, 3.0, Z0 + DT + 1.0));
	difference(&difference(&cam, &bore), &dflat)
}

/// Output flange: a disc above the cycloidal disc with 6 output pins (Ø5.6)
/// dropping through the disc's output holes, and a hub for the arm.
fn output_flange() -> Solid {
	let zf = Z0 + DT; // 14 — flange just above the disc
	let plate = cylinder(DVec3::new(0.0, 0.0, zf), DVec3::Z, 20.0, 4.0, 96); // z 14..18
	let hub = cylinder(DVec3::new(0.0, 0.0, zf + 2.0), DVec3::Z, 12.0, 8.0, 64); // z 16..24, OVERLAPS plate
	let mut f = union(&plate, &hub);
	for i in 0..6 {
		let a = TAU * i as f64 / 6.0;
		// Output pins drop from inside the flange (z16) down through the disc holes; overlap the plate.
		f = union(&f, &cylinder(DVec3::new(12.0 * a.cos(), 12.0 * a.sin(), Z0 - 1.0), DVec3::Z, 2.8, 9.0, 32)); // z 7..16
	}
	f
}

/// The implicit LATTICE ARM (max strength / light weight): a gyroid-filled beam
/// spanning from the output hub out to the tip, meshed from one SDF tree. The
/// bolt-precise NEMA-17 mount face is an EXACT B-rep part (`nema_tip_mount`) bonded
/// at the tip — lattice where we want light+strong, machined-exact where it bolts.
/// Returns the watertight arm mesh.
fn arm_lattice() -> Mesh {
	let zc = (Z0 + DT + 7.0) as f32; // arm centre height ~21, aligned with the flange hub
	let xtip = ARM_LEN as f32; // lattice runs to x=80; the B-rep mount caps 80..92
	// Beam envelope: from the output hub (x~6) out to the tip; 22 (y) x 18 (z) section.
	let beam = || Node::primitive(Cuboid::new(Vec3::new(0.5 * (6.0 + xtip), 0.0, zc), Vec3::new(0.5 * (xtip - 6.0), 11.0, 9.0)));
	// VISIBLE gyroid lattice clipped to the beam — NO enclosing skin, so the pores
	// show and the arm is genuinely light. 9 mm cell reads clearly as a lattice.
	let region = Aabb::new(Vec3::new(-4.0, -14.0, zc - 14.0), Vec3::new(xtip + 8.0, 14.0, zc + 14.0));
	let lattice = Node::primitive(Tpms::network(region, TpmsKind::Gyroid, 9.0, 0.0)).intersection(beam());
	// Solid load interfaces (no holes -> watertight): a hub bonding to the output
	// flange (joint Z-axis) and a tip cap the exact B-rep NEMA mount bolts onto.
	let out_hub = Node::primitive(Cuboid::new(Vec3::new(6.0, 0.0, zc), Vec3::new(14.0, 11.0, 11.0)));
	let tip_cap = Node::primitive(Cuboid::new(Vec3::new(xtip + 1.0, 0.0, zc), Vec3::new(4.0, 11.0, 11.0)));
	let arm = lattice.union(out_hub).union(tip_cap);
	let domain = Aabb::new(Vec3::new(-10.0, -24.0, zc - 24.0), Vec3::new(xtip + 8.0, 24.0, zc + 24.0));
	dual_contour_narrowband(&arm, domain, Resolution::VoxelSize(0.4))
}

/// Exact B-rep ORTHOGONAL NEMA-17 mount at the arm tip: a 42x42x6 plate whose face
/// normal is +X (perpendicular to this joint's Z axis), with the real NEMA-17 bolt
/// interface — Ø22 pilot + 4x M3 on the 31 mm square, all bored along X. The next
/// motor+drive bolts on here, so chaining these gives a multi-axis arm.
fn nema_tip_mount() -> Solid {
	let zc = Z0 + DT + 7.0; // 21 — matches the lattice arm centre
	// Plate spans x 86..92 (caps the lattice tip at x=80..84), y -21..21, z 0..42.
	let plate = kernel_brep::cuboid(DVec3::new(86.0, -21.0, zc - 21.0), DVec3::new(92.0, 21.0, zc + 21.0));
	let mut m = difference(&plate, &cylinder(DVec3::new(85.0, 0.0, zc), DVec3::X, 11.0, 8.0, 64)); // Ø22 pilot
	for (sy, sz) in [(1.0, 1.0), (1.0, -1.0), (-1.0, 1.0), (-1.0, -1.0)] {
		m = difference(&m, &cylinder(DVec3::new(85.0, 15.5 * sy, zc + 15.5 * sz), DVec3::X, 1.7, 8.0, 24)); // 4x M3
	}
	m
}

fn emit(dir: &str, name: &str, s: &Solid, want_wt: bool) -> (bool, Mesh) {
	let v = validate(s);
	let mesh = tessellate_default(s);
	let wt = mesh.is_watertight();
	let _ = std::fs::create_dir_all(dir);
	let _ = std::fs::write(format!("{dir}/{name}.stl"), mesh.to_stl_binary());
	let ok = v.is_valid() && (!want_wt || wt);
	println!("  {name:16} valid={} genus={:2} watertight={wt} vol={:>8.0} mm³  {}", v.is_valid(), v.genus, volume(s).abs(), if ok { "OK" } else { "<<<" });
	(ok, mesh)
}

fn merge_into(dst: &mut Mesh, src: &Mesh) {
	let base = dst.positions.len() as u32;
	dst.positions.extend_from_slice(&src.positions);
	dst.indices.extend(src.indices.iter().map(|i| i + base));
}

fn main() {
	let dir = "cyclo_out";
	println!("CYCLOIDAL DRIVE 10:1 for NEMA 17 (B-rep drive + implicit lattice arm):");
	let disc = cycloid_disc();
	let ring = pin_ring_housing();
	let cam = eccentric_cam();
	let flange = output_flange();

	let mount = nema_tip_mount();
	let (o1, m_disc) = emit(dir, "cycloid_disc", &disc, true);
	let (o2, m_ring) = emit(dir, "pin_ring_housing", &ring, true);
	let (o3, m_cam) = emit(dir, "eccentric_cam", &cam, true);
	let (o4, m_flange) = emit(dir, "output_flange", &flange, true);
	let (o5, m_mount) = emit(dir, "nema_tip_mount", &mount, true);
	let parts_ok = o1 && o2 && o3 && o4 && o5;

	println!("  building implicit lattice arm ...");
	let m_arm = arm_lattice();
	let arm_wt = m_arm.is_watertight();
	let _ = m_arm.write_stl_binary(format!("{dir}/lattice_arm.stl"));
	println!("  lattice_arm     watertight={arm_wt} tris={}  (gyroid-cored, orthogonal NEMA-17 tip)", m_arm.indices.len() / 3);

	let mut asm = Mesh::new();
	for m in [&m_disc, &m_ring, &m_cam, &m_flange, &m_mount, &m_arm] {
		merge_into(&mut asm, m);
	}
	let _ = asm.write_stl_binary(format!("{dir}/ASSEMBLY.stl"));
	println!("merged assembly: {} tris -> {dir}/ASSEMBLY.stl", asm.indices.len() / 3);

	let ratio = (ZP - 1.0) / (ZP - (ZP - 1.0));
	println!("reduction = {ratio:.0}:1 ; modular: arm tip is an ORTHOGONAL NEMA-17 face for the next joint");
	let ok = parts_ok && arm_wt;
	println!("RESULT: {}", if ok { "PASS" } else { "FAIL" });
	if !ok {
		std::process::exit(1);
	}
}
