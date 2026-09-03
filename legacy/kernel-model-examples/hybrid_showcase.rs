// Copyright (c) LMCAD. Licensed under the MIT License.

//! Hybrid flagship: two parts that each need BOTH halves of the kernel at full power.
//!
//! 1. **machine_bolt** — M10×1.5 with a real ISO-form thread. The body (shank + hex
//!    head) is exact B-rep; the thread is a trapezoidal crest swept along an exact
//!    helix. Their union SELF-INTERSECTS, which no exact boolean can stitch — so the
//!    voxel half fuses the two watertight bodies into ONE manifold solid through the
//!    winding-number SDF. Pure B-rep precision where it matters, voxels where exact
//!    arithmetic is impossible.
//!
//! 2. **lattice_mount** — a machined mounting boss on a chamfered flange: one revolve,
//!    a coplanar union, an exact `Surface::Torus` rim fillet, then seven chained
//!    drills (every one of those operations was impossible or explosive before the
//!    2026-06-09 robustness fixes). Then the voxel half wraps the boss in a gyroid
//!    lattice web, smooth-blended into the skin with an SDF fillet union — geometry
//!    no B-rep kernel produces — while the bore, bolt circle and seal face stay
//!    machine-exact, with analytic mass properties off the B-rep.
//!
//! Run with: `cargo run --example hybrid_showcase -p kernel-model --release`
//! Writes `hybrid_out/` (STL for printing, STEP for CAD, 3MF for slicers).

use std::f64::consts::{PI, TAU};

use kernel_brep::math::{DAffine3, DVec2, DVec3};
use kernel_brep::{
	cylinder, difference, export_step, extrude, fillet_circular_rim, mass_properties, revolve, sweep_solid,
	tessellate_adaptive_tol, union, validate, volume, Solid, Surface,
};
use kernel_core::check_mesh;
use kernel_core::mesh::Mesh;
use kernel_implicit::{
	dual_contour_narrowband, fillet_union, make_manifold, manifold_dual_contour, Aabb, Cone as VoxCone,
	Cylinder as VoxCylinder, Gyroid, Node, Resolution, Sdf, Torus as VoxTorus, Vec3,
};

/// Resin-printer-grade extraction cells. Narrow-band dual contouring scales with
/// surface AREA (not volume), and places vertices on the true zero level-set via the
/// gradient QEF — so the surface deviation is far below the cell size; the cell size
/// bounds only the smallest resolvable feature. Every SDF here is CLOSED-FORM (no
/// mesh-sampled winding fields), so there is no sampling noise to dent the surface.
const BOLT_VOXEL: f32 = 0.06;
const MOUNT_VOXEL: f32 = 0.15;

/// Exact implicit ISO thread: in helical coordinates (radius, axial offset from the
/// nearest turn) the swept trapezoidal ridge is a FIXED convex quad, so its signed
/// field is closed form — the max of the quad's four edge half-planes (exact zero
/// set), clamped to the threaded span. The helical unwrap `z − pitch·θ/2π` is
/// continuous across the θ branch cut because the jump is exactly one pitch.
struct HelicalThreadSdf {
	shank_r: f32,
	z0: f32,
	z1: f32,
	pitch: f32,
	depth: f32,
}

impl Sdf for HelicalThreadSdf {
	fn distance(&self, p: Vec3) -> f32 {
		let rad = (p.x * p.x + p.y * p.y).sqrt();
		let theta = p.y.atan2(p.x);
		let mut u = (p.z - self.z0 - self.pitch * theta / std::f32::consts::TAU).rem_euclid(self.pitch);
		if u > self.pitch * 0.5 {
			u -= self.pitch;
		}
		// CCW quad in the (rad, u) plane: root buried 0.3 in the shank, ISO-like
		// near-touching base (0.43·P), small crest flat (0.08·P).
		let (ra, rc) = (self.shank_r - 0.3, self.shank_r + self.depth);
		let (bw, cw) = (self.pitch * 0.43, self.pitch * 0.08);
		let v = [[ra, -bw], [rc, -cw], [rc, cw], [ra, bw]];
		let mut d = f32::NEG_INFINITY;
		for i in 0..4 {
			let (a, b) = (v[i], v[(i + 1) % 4]);
			let (ex, ey) = (b[0] - a[0], b[1] - a[1]);
			let inv_len = 1.0 / (ex * ex + ey * ey).sqrt();
			d = d.max(((rad - a[0]) * ey - (u - a[1]) * ex) * inv_len);
		}
		d.max(self.z0 - p.z).max(p.z - self.z1)
	}

	fn bounds(&self) -> Aabb {
		let r = self.shank_r + self.depth;
		Aabb::from_center_half_extent(
			Vec3::new(0.0, 0.0, (self.z0 + self.z1) * 0.5),
			Vec3::new(r, r, (self.z1 - self.z0) * 0.5 + self.pitch),
		)
	}
}

/// Exact implicit hex prism (across-flats `af`, flats matching `hexagon_af`):
/// the max of six flat half-planes and two end planes — exact zero set.
struct HexPrismSdf {
	af: f32,
	z0: f32,
	z1: f32,
}

impl Sdf for HexPrismSdf {
	fn distance(&self, p: Vec3) -> f32 {
		let mut d = f32::NEG_INFINITY;
		for k in 0..3 {
			let a = k as f32 * std::f32::consts::PI / 3.0;
			d = d.max((p.x * a.cos() + p.y * a.sin()).abs() - self.af * 0.5);
		}
		d.max(self.z0 - p.z).max(p.z - self.z1)
	}

	fn bounds(&self) -> Aabb {
		let r = self.af * 0.5 / (std::f32::consts::PI / 6.0).cos();
		Aabb::from_center_half_extent(Vec3::new(0.0, 0.0, (self.z0 + self.z1) * 0.5), Vec3::new(r, r, (self.z1 - self.z0) * 0.5))
	}
}

/// A regular hexagon by across-flats width (wrench size), flat parallel to X.
fn hexagon_af(width: f64) -> Vec<DVec2> {
	let circumradius = (width * 0.5) / (PI / 6.0).cos();
	(0..6)
		.map(|i| {
			let a = PI / 6.0 + i as f64 * PI / 3.0;
			DVec2::new(circumradius * a.cos(), circumradius * a.sin())
		})
		.collect()
}

/// An ISO-form helical thread for a shank of radius `r`: a trapezoidal crest
/// (root buried in the shank, near-touching turns, small crest flat) swept along an
/// exact helix of the given `pitch` from `z0` over `turns` revolutions.
fn iso_thread(r: f64, z0: f64, pitch: f64, turns: f64, depth: f64) -> Option<Solid> {
	let steps_per_turn = 96;
	let n = (turns * steps_per_turn as f64).round() as usize;
	let path: Vec<DVec3> = (0..=n)
		.map(|k| {
			let t = k as f64 / steps_per_turn as f64;
			let a = t * TAU;
			DVec3::new(r * a.cos(), r * a.sin(), z0 + t * pitch)
		})
		.collect();
	// Trapezoid in the (radial, axial) plane, wound top → crest → bottom so the swept
	// ridge faces outward. Base half-width 0.43·P leaves a thin root land between
	// turns (ISO-like); the root sits 0.3 mm inside the shank so the bodies overlap.
	let (bw, cw) = (pitch * 0.43, pitch * 0.08);
	let profile = vec![
		DVec3::new(r - 0.3, 0.0, z0 + bw),
		DVec3::new(r + depth, 0.0, z0 + cw),
		DVec3::new(r + depth, 0.0, z0 - cw),
		DVec3::new(r - 0.3, 0.0, z0 - bw),
	];
	sweep_solid(&profile, &path)
}

/// Append `src` into `dst` unchanged (the bodies already sit in world space).
fn merge_into(dst: &mut Mesh, src: &Mesh) {
	let base = dst.positions.len() as u32;
	dst.positions.extend_from_slice(&src.positions);
	for t in src.triangles() {
		dst.push_triangle(base + t[0], base + t[1], base + t[2]);
	}
}

/// PART 1 — M10×1.5 hex bolt with a true machine thread, voxel-fused into one solid.
fn machine_bolt(dir: &str) -> bool {
	// Exact B-rep body: Ø10 shank 40 long, AF16 head 6.4 tall (coplanar union). The
	// shank is a 48-gon B-rep — the Surface::Cylinder tag drives adaptive tessellation,
	// so the display mesh is 5 µm smooth regardless of the B-rep segment count.
	let shank = cylinder(DVec3::ZERO, DVec3::Z, 5.0, 40.0, 48);
	let head = extrude(&hexagon_af(16.0), 6.4).transformed(DAffine3::from_translation(DVec3::new(0.0, 0.0, 40.0)));
	let body = union(&shank, &head);
	let vb = validate(&body);

	// ISO-form thread: pitch 1.5, depth 0.85, threaded length 26 mm (z 2..28).
	let thread = iso_thread(5.0, 2.0, 1.5, 26.0 / 1.5, 0.85).expect("thread sweep solidifies");

	// Display mesh: both bodies on the EXACT path at 5 µm — razor-sharp crests.
	let tol = 0.005;
	let mut sharp = tessellate_adaptive_tol(&body, tol);
	let body_wt = sharp.is_watertight();
	let tmesh = tessellate_adaptive_tol(&thread, tol);
	let thread_wt = tmesh.is_watertight();
	merge_into(&mut sharp, &tmesh);
	sharp.write_stl_binary(format!("{dir}/machine_bolt_sharp.stl")).expect("write stl");
	std::fs::write(format!("{dir}/machine_bolt_body.step"), export_step(&body, "machine_bolt_body")).expect("write step");

	// HYBRID FUSE: the exact body∪thread boolean self-intersects (the ridge pierces
	// the shank wall), which no exact arrangement can stitch — but the bolt has an
	// exact IMPLICIT TWIN: shank cylinder ∪ hex prism ∪ the closed-form helical
	// thread field. All ns-per-query closed forms — no mesh-sampled winding SDF, so
	// no sampling noise ("orange peel") on the surface — extracted on a 0.06 mm
	// narrow band. No smoothing pass: the QEF already sits vertices on the true
	// surface, and smoothing would soften the thread crests.
	let bolt_twin = Node::primitive(VoxCylinder::new(Vec3::ZERO, Vec3::new(0.0, 0.0, 40.0), 5.0))
		.union(Node::primitive(HexPrismSdf { af: 16.0, z0: 40.0, z1: 46.4 }))
		.union(Node::primitive(HelicalThreadSdf { shank_r: 5.0, z0: 2.0, z1: 28.0, pitch: 1.5, depth: 0.85 }));
	let bounds = bolt_twin.bounds().pad(1.0);
	let mut fused = dual_contour_narrowband(&bolt_twin, bounds, Resolution::VoxelSize(BOLT_VOXEL));
	if check_mesh(&fused).non_manifold_edges > 0 {
		fused = make_manifold(&fused);
	}
	let r = check_mesh(&fused);
	let fused_ok = fused.is_watertight() && r.non_manifold_edges == 0;
	fused.write_stl_binary(format!("{dir}/machine_bolt_fused.stl")).expect("write stl");

	let exact_sum = volume(&body).abs() + volume(&thread).abs();
	let fused_vol = fused.signed_volume().abs();
	let ok = vb.closed && vb.manifold && vb.genus == 0 && body_wt && thread_wt && fused_ok && (fused_vol - exact_sum).abs() / exact_sum < 0.05;
	println!(
		"  machine_bolt  body[closed={} genus={}] thread_watertight={}  sharp {} tris  FUSED one-manifold={} {} tris  vol {:.0} mm³ (bodies sum {:.0}, overlap makes it slightly less)  {}",
		vb.closed,
		vb.genus,
		thread_wt,
		sharp.triangle_count(),
		fused_ok,
		fused.triangle_count(),
		fused_vol,
		exact_sum,
		if ok { "PASS" } else { "FAIL" }
	);
	ok
}

/// PART 2 — machined mounting boss + flange (pure exact B-rep) wrapped in a gyroid
/// lattice web smooth-blended by the voxel half (pure implicit), fused watertight.
fn lattice_mount(dir: &str) -> bool {
	// --- EXACT HALF: every machined interface is analytic ------------------------
	// One revolved ring: Ø80 flange, 8 thick, 1×45° chamfer drawn into the profile.
	// (64-segment B-reps: the analytic surface tags drive adaptive tessellation, so
	// display smoothness is independent of segment count — and the probe showed the
	// fillet-first ordering below is run-to-run stable at every count, while
	// 96-segment union-first sat on the R5 nondeterminism margin.)
	let flange = revolve(
		&[DVec2::new(10.0, 0.0), DVec2::new(40.0, 0.0), DVec2::new(40.0, 7.0), DVec2::new(39.0, 8.0), DVec2::new(10.0, 8.0)],
		64,
	);
	// Ø36 boss with its top rim filleted FIRST — an exact Surface::Torus band on the
	// seal-face edge — then seated coplanar on the flange by the exact boolean.
	let boss = cylinder(DVec3::new(0.0, 0.0, 8.0), DVec3::Z, 18.0, 28.0, 64);
	let boss = fillet_circular_rim(&boss, DVec3::new(18.0, 0.0, 36.0), 2.5, 8).expect("boss rim fillets to an exact torus");
	let m = union(&flange, &boss);
	// THEN drill: Ø20 precision bore through everything + 6 × Ø6 bolt circle at R30 —
	// seven chained cuts into already-holed faces (bug R2 territory, now solid).
	let mut m = difference(&m, &cylinder(DVec3::new(0.0, 0.0, -1.0), DVec3::Z, 10.0, 38.5, 64));
	for i in 0..6 {
		let a = i as f64 * PI / 3.0;
		m = difference(&m, &cylinder(DVec3::new(30.0 * a.cos(), 30.0 * a.sin(), -1.0), DVec3::Z, 3.0, 10.0, 24));
	}
	let v = validate(&m);
	let torus_faces = m.faces().filter(|&f| matches!(m.face(f).surface, Surface::Torus { .. })).count();
	let mp = mass_properties(&m);
	let skin = tessellate_adaptive_tol(&m, 0.02);
	let skin_wt = skin.is_watertight();
	skin.write_stl_binary(format!("{dir}/lattice_mount_skin.stl")).expect("write stl");
	std::fs::write(format!("{dir}/lattice_mount_skin.step"), export_step(&m, "lattice_mount_skin")).expect("write step");
	println!(
		"  mount skin    closed={} manifold={} genus={} (want 7)  {} exact torus faces  vol={:.0} mm³ CoM z={:.2} mm Izz={:.0} mm⁵ (analytic)  {} tris watertight={}",
		v.closed,
		v.manifold,
		v.genus,
		torus_faces,
		mp.volume,
		mp.center_of_mass.z,
		mp.inertia.z_axis.z,
		skin.triangle_count(),
		skin_wt
	);

	// --- VOXEL HALF at RESIN resolution: an exact implicit TWIN of the skin -------
	// Every machined face of this part is analytic (cylinders, a chamfer cone, a
	// torus), so instead of sampling the tessellation through a winding-number SDF
	// (µs per query) the voxel half mirrors the part with exact SDF primitives
	// (ns per query) — which makes a 0.12 mm narrow-band extraction affordable.
	let zcyl = |z0: f32, z1: f32, r: f32| {
		Node::primitive(VoxCylinder::new(Vec3::new(0.0, 0.0, z0), Vec3::new(0.0, 0.0, z1), r))
	};
	let twin = || {
		// Flange Ø80×8, its 1×45° rim chamfer cut by an outside-cone ring tool.
		let chamfer_ring = zcyl(7.0, 8.2, 41.0)
			.difference(Node::primitive(VoxCone { a: Vec3::new(0.0, 0.0, 7.0), b: Vec3::new(0.0, 0.0, 8.2), ra: 40.0, rb: 38.8 }));
		// Boss with the torus-rounded top rim (rim circle R15.5 at z33.5, tube r2.5).
		let boss = zcyl(8.0, 33.5, 18.0)
			.union(zcyl(8.0, 36.0, 15.5))
			.union(Node::primitive(VoxTorus::new(Vec3::new(0.0, 0.0, 33.5), Vec3::Z, 15.5, 2.5)));
		let holes = (0..6).fold(zcyl(-1.0, 39.0, 10.0), |acc, i| {
			let a = i as f32 * std::f32::consts::PI / 3.0;
			acc.union(Node::primitive(VoxCylinder::new(
				Vec3::new(30.0 * a.cos(), 30.0 * a.sin(), -1.0),
				Vec3::new(30.0 * a.cos(), 30.0 * a.sin(), 10.0),
				3.0,
			)))
		});
		zcyl(0.0, 8.0, 40.0).difference(chamfer_ring).union(boss).difference(holes)
	};
	// Web region: a frustum shroud from the flange top up the boss, keeping clear of
	// the precision bore (subtract an oversize core column so the bore stays exact).
	let web = || {
		Node::primitive(VoxCone { a: Vec3::new(0.0, 0.0, 8.0), b: Vec3::new(0.0, 0.0, 34.0), ra: 27.0, rb: 14.0 })
			.difference(zcyl(-1.0, 40.0, 10.8))
	};
	let gy_region = Aabb::from_center_half_extent(Vec3::new(0.0, 0.0, 21.0), Vec3::new(28.0, 28.0, 14.0));
	let lattice = Node::primitive(Gyroid::new(gy_region, 0.5, 1.6)).intersection(web());

	// FUSION: smooth-blend the lattice into the twin (radius-2 SDF fillet — the
	// hybrid op neither half has alone) and extract at resin resolution with the
	// MANIFOLD dual contour: the blend grazes the lattice tangentially in thousands
	// of places at this resolution, and only the manifold mesher resolves those
	// pinch saddles (the narrow-band surface-nets variant left ~800 pinch edges).
	// A tight explicit domain (the conservative auto-bounds waste a 40 mm empty
	// slab) keeps the dense lattice affordable. No smoothing pass afterwards: the
	// QEF puts vertices on the true surface.
	let hybrid = fillet_union(twin(), lattice, 2.0);
	let bounds = Aabb::from_center_half_extent(Vec3::new(0.0, 0.0, 19.0), Vec3::new(42.5, 42.5, 20.5));
	let mut hmesh = manifold_dual_contour(&hybrid, bounds, Resolution::VoxelSize(MOUNT_VOXEL));
	// A residual TPMS pinch can survive any fixed grid: snip it apart (geometry-
	// preserving); if cracks remain, re-extract on a slightly shifted grid — cheap,
	// because the twin is pure closed-form SDF.
	if check_mesh(&hmesh).non_manifold_edges > 0 || !hmesh.is_watertight() {
		hmesh = make_manifold(&hmesh);
		if check_mesh(&hmesh).non_manifold_edges > 0 || !hmesh.is_watertight() {
			hmesh = manifold_dual_contour(&hybrid, bounds.pad(MOUNT_VOXEL), Resolution::VoxelSize(MOUNT_VOXEL * 1.07));
		}
	}
	let hr = check_mesh(&hmesh);
	let hvol = hmesh.signed_volume().abs();
	hmesh.write_stl_binary(format!("{dir}/lattice_mount_hybrid.stl")).expect("write stl");
	hmesh.write_3mf(format!("{dir}/lattice_mount_hybrid.3mf")).expect("write 3mf");

	// What did the lattice buy? Compare against the same part with a SOLID web
	// (coarser extraction — it only feeds a volume number).
	let solid_web = fillet_union(twin(), web(), 2.0);
	let svol = dual_contour_narrowband(&solid_web, bounds, Resolution::VoxelSize(0.3)).signed_volume().abs();
	let saved = (svol - hvol) / (svol - mp.volume).max(1e-9) * 100.0;

	let ok = v.closed && v.manifold && v.genus == 7 && torus_faces > 0 && skin_wt && hmesh.is_watertight() && hr.non_manifold_edges == 0 && hvol > mp.volume;
	println!(
		"  hybrid mount  {} tris  watertight={} non-manifold={}  vol {:.0} mm³ (skin {:.0} + web)  lattice web saves {:.0}% of the solid-web material  {}",
		hmesh.triangle_count(),
		hmesh.is_watertight(),
		hr.non_manifold_edges,
		hvol,
		mp.volume,
		saved,
		if ok { "PASS" } else { "FAIL" }
	);
	ok
}

fn main() {
	let dir = "hybrid_out";
	std::fs::create_dir_all(dir).expect("create output dir");
	println!("Hybrid showcase — exact B-rep + voxel half, each doing what only it can:\n");
	let ok = machine_bolt(dir) & lattice_mount(dir);
	println!("\n{} — wrote files to ./{dir}/", if ok { "ALL PASS" } else { "FAILED" });
	std::process::exit(if ok { 0 } else { 1 });
}
