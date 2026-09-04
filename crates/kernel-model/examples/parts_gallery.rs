// Copyright (c) LMCAD. Licensed under the MIT License.

//! Parts gallery: six real engineering parts built end-to-end with the hybrid kernel,
//! each validated and written to `parts_out/` as STL (plus STEP for the exact B-rep
//! parts and 3MF for the lattice). Together they exercise both halves of the kernel:
//!
//! 1. **flange** — one revolved exact cross-section minus a 6-hole bolt circle.
//! 2. **spur_gear** — a real 20-tooth involute gear profile, extruded, bored, keyed.
//! 3. **bracket** — named-edge corner fillets + coplanar face-sharing unions + holes.
//! 4. **enclosure** — drafted (2°) molding walls, DFM draft/thickness analysis on the
//!    exact body, then hollowed to a 2 mm shell through the voxel half.
//! 5. **gyroid_block** — a watertight TPMS lattice via Manifold Dual Contouring.
//! 6. **fastener_stack** — bolt + washer + nut placed as an assembly: clearance,
//!    interference and mass-property analysis, plus a merged display mesh.
//!
//! Run with: `cargo run --example parts_gallery -p kernel-model --release`

use std::f64::consts::PI;

use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{
	cuboid, cylinder, difference, draft_analysis, exact_volume, export_step, extrude, extrude_tapered, fillet_edge, mass_properties,
	revolve, tessellate_adaptive_tol, union, validate, volume, wall_thickness, EdgeName, FaceName, FaceSource, Solid,
};
use kernel_core::check_mesh;
use kernel_core::math::Affine3A;
use kernel_core::mesh::Mesh;
use kernel_implicit::{
	dual_contour_narrowband, make_manifold, Aabb, Cuboid as VoxCuboid, Gyroid, Node, Plane as VoxPlane, Resolution, Vec3,
};
use kernel_model::parts::{hex_bolt, hex_nut, washer};
use kernel_model::{watertight_mesh, Assembly, Instance};

const TOL: f64 = 0.01; // 10 µm chord tolerance for the exact tessellation path

/// The flange cross-section in the (radius, z) half-plane, wound CCW:
/// disc r10→40 over z0→6, raised neck r10→18 over z6→18, Ø20 bore through.
const FLANGE_PROFILE: [DVec2; 6] = [
	DVec2::new(10.0, 0.0),
	DVec2::new(40.0, 0.0),
	DVec2::new(40.0, 6.0),
	DVec2::new(18.0, 6.0),
	DVec2::new(18.0, 18.0),
	DVec2::new(10.0, 18.0),
];

/// PART 1 — pipe flange, built the NATURAL way: one revolved L-cross-section (disc +
/// raised neck + Ø20 through-bore), then a 6 × Ø6 bolt circle drilled as chained
/// boolean differences. This exact recipe was IMPOSSIBLE before 2026-06-09: the
/// concave-profile revolve was born invalid (bug R1) and every successive hole
/// compounded the explosion (bug R2, genus walked 129→457). Now valid by
/// construction. Genus 7 (bore + six bolt holes).
fn flange() -> Solid {
	let mut f = revolve(&FLANGE_PROFILE, 96);
	for i in 0..6 {
		let a = i as f64 * PI / 3.0;
		let hole = cylinder(DVec3::new(30.0 * a.cos(), 30.0 * a.sin(), -1.0), DVec3::Z, 3.0, 8.0, 24);
		f = difference(&f, &hole);
	}
	f
}

/// One point of an involute unrolled from a base circle of radius `rb`, at roll
/// parameter `t` (radians of unwound arc).
fn involute(rb: f64, t: f64) -> DVec2 {
	DVec2::new(rb * (t.cos() + t * t.sin()), rb * (t.sin() - t * t.cos()))
}

fn rot(p: DVec2, a: f64) -> DVec2 {
	DVec2::new(p.x * a.cos() - p.y * a.sin(), p.x * a.sin() + p.y * a.cos())
}

/// PART 2 — a real involute spur gear: module 2, 20 teeth, 20° pressure angle, 8 mm
/// face width, Ø10 bore with a 3.3 mm keyway. The tooth flanks are true involutes of
/// the base circle (sampled into the profile polygon); root and tip lands are arcs.
fn spur_gear() -> Solid {
	let (m, z, fw) = (2.0f64, 20usize, 8.0);
	let alpha = 20f64.to_radians();
	let rp = m * z as f64 / 2.0; // pitch radius 20
	let rb = rp * alpha.cos(); // base radius
	let ra = rp + m; // addendum (tip) radius 22
	let rr = rp - 1.25 * m; // root radius 17.5
	let t_tip = ((ra / rb).powi(2) - 1.0).sqrt(); // roll parameter where the involute reaches the tip
	let theta_tip = t_tip - t_tip.atan(); // polar spread of the involute root→tip
									   // Angular half-width of a tooth at the base circle: half the pitch-circle tooth
									   // thickness (π/2z) plus the involute spread inv(α) rolled back from pitch to base.
	let half = PI / (2.0 * z as f64) + (alpha.tan() - alpha);
	let pitch = 2.0 * PI / z as f64;

	let mut poly: Vec<DVec2> = Vec::new();
	for k in 0..z {
		let c = k as f64 * pitch;
		let (a_l, a_r) = (c - half, c + half);
		// Root land: an arc at rr spanning the gap from the previous tooth's right
		// flank to this tooth's left flank (the radial joins are polygon edges).
		for j in 0..4 {
			let a = a_l - (pitch - 2.0 * half) * (1.0 - j as f64 / 3.0);
			poly.push(DVec2::new(rr * a.cos(), rr * a.sin()));
		}
		// Left flank: involute from the base circle out to the tip.
		for j in 0..=7 {
			poly.push(rot(involute(rb, t_tip * j as f64 / 7.0), a_l));
		}
		// Tip land: an arc at ra between the two flank tips.
		for j in 1..=2 {
			let a = (a_l + theta_tip) + ((a_r - theta_tip) - (a_l + theta_tip)) * j as f64 / 3.0;
			poly.push(DVec2::new(ra * a.cos(), ra * a.sin()));
		}
		// Right flank: the mirrored involute, traversed tip→base so the angle ascends.
		for j in (0..=7).rev() {
			let i = involute(rb, t_tip * j as f64 / 7.0);
			poly.push(rot(DVec2::new(i.x, -i.y), a_r));
		}
	}

	// Drill the bore, then broach the keyway THROUGH the bore wall — the cut-across-
	// a-cut-curved-wall case that exploded before 2026-06-09 (bug R3). Now the
	// natural two-boolean recipe holds.
	let blank = extrude(&poly, fw);
	let bore = cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, 5.0, fw + 2.0, 48);
	let key = cuboid(DVec3::new(2.0, -1.65, -1.0), DVec3::new(6.7, 1.65, fw + 1.0));
	difference(&difference(&blank, &bore), &key)
}

/// PART 3 — ribbed L-bracket: base plate with its two free corners rounded by
/// NAMED-EDGE fillets (persistent topological names), an upright wall sharing the
/// plate's face planes (the coplanar-boolean case), a triangular gusset rib seated
/// face-to-face on both, and TWO Ø7 mounting holes drilled sequentially through the
/// same plate face — the chained-boolean case that exploded before the loop-aware
/// arrangement fix (bug R2, fixed 2026-06-09). Genus 2.
fn bracket() -> Solid {
	let plate = cuboid(DVec3::ZERO, DVec3::new(60.0, 40.0, 8.0));
	// Cuboid faces are indexed with 5=+X, 3=+Y, 2=−Y (see the kernel-brep fillet
	// tests) — the two vertical edges at x=60 are (+X,+Y) and (+X,−Y).
	let f = |i: u32| FaceName { operand: FaceSource::Primitive, source_face: i };
	let plate = fillet_edge(&plate, EdgeName::new(f(5), f(3)), 10.0).expect("+X+Y corner fillet");
	let plate = fillet_edge(&plate, EdgeName::new(f(5), f(2)), 10.0).expect("+X−Y corner fillet");

	let upright = cuboid(DVec3::ZERO, DVec3::new(8.0, 40.0, 50.0));
	// Gusset: a right triangle drawn in the XZ plane, extruded 6 mm and rotated up so
	// its base sits ON the plate top and its back ON the upright face (face contact).
	let rib = extrude(&[DVec2::new(8.0, 8.0), DVec2::new(40.0, 8.0), DVec2::new(8.0, 40.0)], 6.0)
		.transformed(DAffine3::from_translation(DVec3::new(0.0, 23.0, 0.0)) * DAffine3::from_rotation_x(PI / 2.0));

	let mut b = union(&union(&plate, &upright), &rib);
	for y in [12.0, 28.0] {
		b = difference(&b, &cylinder(DVec3::new(45.0, y, -1.0), DVec3::Z, 3.5, 10.0, 32));
	}
	b
}

/// PART 4 — drafted enclosure: a 50×30×25 body with 2° drafted walls (one exact
/// tapered extrusion). DFM is checked on the exact B-rep (draft + wall thickness),
/// then the body is hollowed to a 2 mm shell through the VOXEL half at RESIN
/// resolution: the drafted box is a convex intersection of six half-spaces — an
/// EXACT implicit twin (closed-form SDF, ns queries) — so the shell (body minus its
/// exact inward offset) extracts on a 0.10 mm narrow band with mathematically
/// parallel 2.00 mm walls.
fn enclosure() -> (Solid, Mesh) {
	let outer = extrude_tapered(
		&[DVec2::new(-25.0, -15.0), DVec2::new(25.0, -15.0), DVec2::new(25.0, 15.0), DVec2::new(-25.0, 15.0)],
		25.0,
		2f64.to_radians(),
	);
	let d = 2f32.to_radians();
	let plane = |p: [f32; 3], n: [f32; 3]| Node::primitive(VoxPlane::new(Vec3::from(p), Vec3::from(n).normalize()));
	let twin = || {
		plane([0.0, 0.0, 0.0], [0.0, 0.0, -1.0])
			.intersection(plane([0.0, 0.0, 25.0], [0.0, 0.0, 1.0]))
			.intersection(plane([25.0, 0.0, 0.0], [d.cos(), 0.0, d.sin()]))
			.intersection(plane([-25.0, 0.0, 0.0], [-d.cos(), 0.0, d.sin()]))
			.intersection(plane([0.0, 15.0, 0.0], [0.0, d.cos(), d.sin()]))
			.intersection(plane([0.0, -15.0, 0.0], [0.0, -d.cos(), d.sin()]))
	};
	let shell_node = twin().difference(twin().offset(-2.0));
	let bounds = Aabb::from_center_half_extent(Vec3::new(0.0, 0.0, 12.5), Vec3::new(27.0, 17.0, 14.5));
	let mut shell = dual_contour_narrowband(&shell_node, bounds, Resolution::VoxelSize(0.10));
	if check_mesh(&shell).non_manifold_edges > 0 || !shell.is_watertight() {
		shell = make_manifold(&shell);
	}
	(outer, shell)
}

/// PART 5 — gyroid TPMS lattice block: 40 mm cube of 0.6-thick gyroid shell at RESIN
/// resolution — a 0.15 mm narrow band puts ~4 cells across each wall, with the
/// snip-or-shift remedy for the stray TPMS saddle pinch.
fn gyroid_block() -> Mesh {
	let half = 20.0;
	let region = Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(half));
	let lattice =
		Node::primitive(Gyroid::new(region, 0.35, 0.6)).intersection(Node::primitive(VoxCuboid::new(Vec3::ZERO, Vec3::splat(half))));
	let mut m = dual_contour_narrowband(&lattice, region.pad(0.5), Resolution::VoxelSize(0.15));
	if check_mesh(&m).non_manifold_edges > 0 || !m.is_watertight() {
		m = make_manifold(&m);
		if check_mesh(&m).non_manifold_edges > 0 || !m.is_watertight() {
			m = dual_contour_narrowband(&lattice, region.pad(0.7), Resolution::VoxelSize(0.16));
		}
	}
	m
}

/// Append `src` into `dst`, translating its vertices by `offset`.
fn merge_into(dst: &mut Mesh, src: &Mesh, offset: DVec3) {
	let base = dst.positions.len() as u32;
	let off = offset.as_vec3();
	for p in &src.positions {
		dst.positions.push(*p + off);
	}
	for t in src.triangles() {
		dst.push_triangle(base + t[0], base + t[1], base + t[2]);
	}
}

/// True when a mesh is a closed orientable 2-manifold with no collapsed facets
/// or proper triangle crossings. Contact/overlap diagnostics remain available in
/// `check_mesh`, but edge/vertex grazes are not treated as crossings here.
fn manufacturing_ready(mesh: &Mesh) -> bool {
	let report = check_mesh(mesh);
	report.watertight && report.degenerate_triangles == 0 && mesh.self_intersection_witness().is_none()
}

/// Write and re-read the actual manufacturing bytes. A failed round trip is
/// removed so the gallery can never leave a defective artifact behind a PASS.
fn write_manufacturing_mesh(mesh: &Mesh, path: &str) -> bool {
	let p = std::path::Path::new(path);
	let ext = p.extension().and_then(|x| x.to_str()).unwrap_or("").to_ascii_lowercase();
	let written = match ext.as_str() {
		"stl" => mesh.write_stl_binary(p),
		"3mf" => mesh.write_3mf(p),
		_ => return false,
	};
	if written.is_err() {
		let _ = std::fs::remove_file(p);
		return false;
	}
	let mut loaded = match ext.as_str() {
		"stl" => Mesh::read_stl(p),
		"3mf" => Mesh::read_3mf(p),
		_ => unreachable!(),
	};
	let Ok(ref mut round_trip) = loaded else {
		let _ = std::fs::remove_file(p);
		return false;
	};
	round_trip.weld(1.0e-4);
	round_trip.compute_normals();
	if !manufacturing_ready(round_trip) {
		let _ = std::fs::remove_file(p);
		return false;
	}
	true
}

/// Mesh a solid for display/printing: use the exact analytic tessellation only
/// when it passes the manufacturing gate; otherwise heal through the voxel half.
fn part_mesh(solid: &Solid) -> (Mesh, &'static str) {
	let exact = tessellate_adaptive_tol(solid, TOL);
	if manufacturing_ready(&exact) {
		(exact, "exact")
	} else {
		let mut healed = watertight_mesh(solid, 0.3);
		healed.weld(1e-4);
		healed.compute_normals();
		(healed, "voxel-healed")
	}
}

/// Validate, report one table row, and write `<name>.stl` + `<name>.step`.
/// Returns false if any structural check failed.
fn emit(dir: &str, name: &str, solid: &Solid, want_genus: i64) -> bool {
	let v = validate(solid);
	let (mesh, route) = part_mesh(solid);
	let mesh_ok = manufacturing_ready(&mesh);
	let ok = v.closed && v.manifold && v.genus == want_genus && mesh_ok;
	println!(
		"  {:<13} closed={} manifold={} genus={} (want {})  vol={:>9.1} mm³ (exact {:>9.1})  {:>6} tris via {:<12} manufacturing_ready={}  {}",
		name,
		v.closed,
		v.manifold,
		v.genus,
		want_genus,
		volume(solid).abs(),
		exact_volume(solid).abs(),
		mesh.triangle_count(),
		route,
		mesh_ok,
		if ok { "PASS" } else { "FAIL" },
	);
	if ok && !write_manufacturing_mesh(&mesh, &format!("{dir}/{name}.stl")) {
		return false;
	}
	std::fs::write(format!("{dir}/{name}.step"), export_step(solid, name)).expect("write step");
	ok
}

fn main() {
	let dir = "parts_out";
	std::fs::create_dir_all(dir).expect("create output dir");
	println!("Building the parts gallery with the hybrid kernel:\n");
	let mut all_ok = true;

	// 1) Flange — two analytic cross-checks. The FACETED expectation: a revolved
	// polygon at N segments has volume N·sin(2π/N)·M/6 (M = Σ(rᵢ+rⱼ)(rᵢzⱼ−rⱼzᵢ)),
	// minus six 24-gon hole prisms. The TRUE expectation: Pappus 2π·M/6 minus six
	// exact π·r²·h holes — exact_volume should recover it from the surface tags even
	// though the mesh is faceted and the part went through 6 chained booleans.
	let fl = flange();
	let m: f64 = FLANGE_PROFILE.iter().zip(FLANGE_PROFILE.iter().cycle().skip(1)).map(|(a, b)| (a.x + b.x) * (a.x * b.y - b.x * a.y)).sum();
	let ngon = |r: f64, n: f64| n * r * r * (2.0 * PI / n).sin() / 2.0;
	let fl_faceted = 96.0 * (2.0 * PI / 96.0).sin() * m / 6.0 - 6.0 * ngon(3.0, 24.0) * 6.0;
	let fl_true = 2.0 * PI * m / 6.0 - 6.0 * PI * 9.0 * 6.0;
	all_ok &= emit(dir, "flange", &fl, 7);
	println!(
		"                 faceted closed-form {fl_faceted:.1} mm³ → volume() off {:.5}%;  TRUE π-volume {fl_true:.1} mm³ → exact_volume() off {:.5}%",
		(volume(&fl).abs() - fl_faceted).abs() / fl_faceted * 100.0,
		(exact_volume(&fl).abs() - fl_true).abs() / fl_true * 100.0
	);

	// 2) Involute spur gear.
	all_ok &= emit(dir, "spur_gear", &spur_gear(), 1);

	// 3) Ribbed bracket (named-edge fillets + coplanar unions + two chained holes).
	all_ok &= emit(dir, "bracket", &bracket(), 2);

	// 4) Drafted enclosure + DFM + voxel-half hollow.
	let (body, shell) = enclosure();
	all_ok &= emit(dir, "enclosure_body", &body, 0);
	let draft = draft_analysis(&body, DVec3::Z, 1.0);
	let solid_walls = wall_thickness(&body, 1.0);
	let shell_walls = shell.wall_thickness(1.0);
	// The exact-offset cavity has SHARP inner corners (mathematically correct), and
	// the per-triangle inward-ray metric reads oblique distances on the few
	// corner-bevel facets — so judge the wall by quantiles, not the raw minimum
	// (which is reported honestly alongside).
	let mut th: Vec<f64> = shell_walls.thickness.iter().copied().filter(|t| t.is_finite()).collect();
	th.sort_by(f64::total_cmp);
	let (p01, median) = (th[th.len() / 100], th[th.len() / 2]);
	let shell_ok = manufacturing_ready(&shell) && p01 > 1.8 && (1.9..=2.1).contains(&median);
	all_ok &= shell_ok;
	println!(
		"                 DFM: min draft {:.2}° (≥1° area below: {:.1} mm², undercuts: {:.1} mm²); body min wall {:.1} mm",
		draft.min_draft_deg, draft.low_draft_area, draft.undercut_area, solid_walls.min_thickness
	);
	println!(
		"  {:<13} hollowed via voxel half: {} tris, watertight={}, wall median {:.3} / p1 {:.2} mm (target 2.0; raw min {:.2} = sharp-corner ray artifact)  {}",
		"enclosure",
		shell.triangle_count(),
		shell.is_watertight(),
		median,
		p01,
		shell_walls.min_thickness,
		if shell_ok { "PASS" } else { "FAIL" },
	);
	if shell_ok {
		all_ok &= write_manufacturing_mesh(&shell, &format!("{dir}/enclosure_shell.stl"));
	}

	// 5) Gyroid lattice block.
	let gy = gyroid_block();
	let gr = check_mesh(&gy);
	let gy_vol = gy.signed_volume().abs();
	let gy_ok = manufacturing_ready(&gy);
	all_ok &= gy_ok;
	println!(
		"  {:<13} {} tris, watertight={}, non-manifold edges={}, fill {:.1}% of the 40 mm cube  {}",
		"gyroid_block",
		gy.triangle_count(),
		gy.is_watertight(),
		gr.non_manifold_edges,
		gy_vol / (40.0f64.powi(3)) * 100.0,
		if gy_ok { "PASS" } else { "FAIL" },
	);
	if gy_ok {
		all_ok &= write_manufacturing_mesh(&gy, &format!("{dir}/gyroid_block.stl"));
		all_ok &= write_manufacturing_mesh(&gy, &format!("{dir}/gyroid_block.3mf"));
	}

	// 6) Fastener stack assembly: bolt + washer + nut on the shank.
	let bolt = hex_bolt(16.0, 8.0, 10.0, 40.0);
	let wash = washer(22.0, 10.6, 2.5);
	let nut = hex_nut(16.0, 8.0, 10.6);
	let bolt_props = mass_properties(&bolt);
	let (mb, _) = part_mesh(&bolt);
	let (mw, _) = part_mesh(&wash);
	let (mn, _) = part_mesh(&nut);

	let mut asm = Assembly::new();
	asm.add(Instance::from_mesh(&mb, Affine3A::IDENTITY));
	asm.add(Instance::from_mesh(&mw, Affine3A::from_translation(Vec3::new(0.0, 0.0, 2.0))));
	asm.add(Instance::from_mesh(&mn, Affine3A::from_translation(Vec3::new(0.0, 0.0, 5.5))));
	let gap = asm.clearance(0, 1, Resolution::VoxelSize(0.2));
	let hits = asm.interferences(0.05, Resolution::VoxelSize(0.2));
	let total = asm.mass_properties(Resolution::VoxelSize(0.4));
	let mut merged = mb.clone();
	merge_into(&mut merged, &mw, DVec3::new(0.0, 0.0, 2.0));
	merge_into(&mut merged, &mn, DVec3::new(0.0, 0.0, 5.5));
	let asm_ok = hits.is_empty() && gap > 0.05 && gap < 0.6 && manufacturing_ready(&merged);
	all_ok &= asm_ok;
	println!(
		"  {:<13} bolt CoM z={:.2} mm (analytic), washer↔shank clearance {:.2} mm (nominal 0.30), interferences: {}, stack volume {:.0} mm³  {}",
		"fastener_stack",
		bolt_props.center_of_mass.z,
		gap,
		hits.len(),
		total.volume,
		if asm_ok { "PASS" } else { "FAIL" },
	);
	if asm_ok {
		all_ok &= write_manufacturing_mesh(&merged, &format!("{dir}/fastener_stack.stl"));
	}

	println!("\n{} — wrote STL/STEP/3MF files to ./{dir}/", if all_ok { "ALL PARTS PASS" } else { "SOME PARTS FAILED" });
	std::process::exit(if all_ok { 0 } else { 1 });
}

#[cfg(test)]
mod tests {
	use super::*;

	#[test]
	fn gallery_flange_uses_a_manufacturing_ready_mesh() {
		let (mesh, _route) = part_mesh(&flange());
		let report = check_mesh(&mesh);
		assert!(
			report.watertight && report.degenerate_triangles == 0 && mesh.self_intersection_witness().is_none(),
			"gallery export must be a closed orientable manifold without collapsed or crossing triangles: {report:?}"
		);
	}
}
