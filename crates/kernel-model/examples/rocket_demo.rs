// Copyright (c) LMCAD. Licensed under the MIT License.

//! Rocket thrust-chamber demo — the W3 PicoGK-parity primitives doing real
//! computational engineering on ONE part that needs both halves of the kernel.
//!
//! **Exact B-rep half** (the machined interfaces): a bell-nozzle wall — chamber,
//! converging cone, throat and parabolic bell drawn as one revolved profile —
//! plus a mounting flange whose top rim is an exact `Surface::Torus` fillet
//! (rim filleted FIRST on the bare cylinder, then the coplanar-free union, then
//! nine chained drills: the injector-seat counterbore and an 8-bolt circle).
//! Validation, analytic mass properties and the watertight adaptive skin all
//! run on the exact body.
//!
//! **Voxel half** (geometry no B-rep kernel produces): the part's implicit twin
//! is rebuilt from CLOSED-FORM fields — `RevolvedPolygonSdf` (the exact signed
//! distance of a solid of revolution, see its Lipschitz note) mirrors the wall,
//! flange and a cooling-jacket shell; the exact torus mirrors the rim fillet.
//! Then the W3 primitives go to work:
//! - [`Pipe`] carves **conformal cooling channels**: a radius-tapered spiral
//!   that follows the bell mid-wall (tightest at the throat, where the heat
//!   flux peaks) and a `Pipe::helix` around the chamber neck — both subtracted,
//!   both fully embedded in the 4 mm wall.
//! - [`BeamLattice`] fills the jacket cavity between the bell wall and the
//!   outer shell with a conformal pin-fin truss (tapered radial pins +
//!   X-braces, an explicit node/strut graph), welded on with a `fillet_union`
//!   blend — then the whole hybrid is extracted at 0.15 mm with Manifold Dual
//!   Contouring (+ the snip/shift remedy for residual grid-pinch saddles).
//!
//! The channels are verified HOLLOW by physics, not by eyeball: two narrow-band
//! extractions on the SAME grid (channel-less twin vs. carved twin) make the
//! outer-surface quantization cancel in the volume difference, which must match
//! the pipes' analytic tube volume within the documented 5 %.
//!
//! Run with: `cargo run --example rocket_demo -p kernel-model --release`
//! Writes `rocket_out/` (skin STL/STEP, hybrid STL/3MF). Exits non-zero on FAIL.

use std::f64::consts::{PI, TAU};

use kernel_brep::math::{DVec2, DVec3};
use kernel_brep::{
	cylinder, difference, export_step, fillet_circular_rim, mass_properties, revolve, tessellate_adaptive_tol, union,
	validate, Surface,
};
use kernel_core::check_mesh;
use kernel_core::math::Vec2;
use kernel_implicit::{
	dual_contour_narrowband, fillet_union, make_manifold, manifold_dual_contour, Aabb, BeamLattice,
	Cylinder as VoxCylinder, Node, Pipe, Resolution, Sdf, Torus as VoxTorus, Vec3,
};

/// Resin-grade hybrid extraction cell (spec band 0.12–0.15 mm). Narrow-band /
/// manifold DC place vertices on the true zero set via the gradient QEF, so
/// this bounds the smallest resolvable feature, not the surface deviation.
const VOXEL: f32 = 0.15;
/// Cell for the hollowness verification PAIR — both fields are sampled on the
/// identical grid so the outer-surface quantization cancels in the difference;
/// the channels (Ø2 – 2.8 mm) stay ~13 cells wide.
const VERIFY_VOXEL: f32 = 0.18;
/// Wall thickness of the bell/chamber (mm). Channels are buried mid-wall.
const WALL: f64 = 4.0;

/// Inner wall radius (mm) of the engine contour at height `z`: parabolic bell
/// from the Ø44 exit (z = 0) to the Ø18 throat (z 28..33), conical convergence
/// to the Ø36 chamber (z 43..56). Single source of truth for the B-rep
/// profile, the implicit twin and the conformal channel paths.
fn r_inner(z: f64) -> f64 {
	if z <= 28.0 {
		9.0 + 13.0 * ((28.0 - z) / 28.0).powf(1.6)
	} else if z <= 33.0 {
		9.0
	} else if z <= 43.0 {
		9.0 + 9.0 * (z - 33.0) / 10.0
	} else {
		18.0
	}
}

/// z-stations of the wall profile, top (seal land) → bottom (exit lip). The
/// flat-radius spans (chamber, throat, cone ends) are exact two-point segments;
/// the bell is sampled every ~3 mm.
fn wall_stations() -> Vec<f64> {
	let mut zs = vec![56.0, 43.0, 33.0, 28.0];
	zs.extend([26.0, 23.0, 20.0, 17.0, 14.0, 11.0, 8.0, 5.0, 2.0, 0.0]);
	zs
}

/// Closed wall cross-section in the (r, z) half-plane: down the inner contour,
/// across the exit lip, back up the outer contour (`r_inner + WALL`).
fn wall_profile() -> Vec<DVec2> {
	let zs = wall_stations();
	let mut pts: Vec<DVec2> = zs.iter().map(|&z| DVec2::new(r_inner(z), z)).collect();
	pts.extend(zs.iter().rev().map(|&z| DVec2::new(r_inner(z) + WALL, z)));
	pts
}

/// Exact solid of revolution about Z of a simple profile polygon in the
/// (radius, z) half-plane (all radii ≥ 0, asserted).
///
/// **1-Lipschitz / exactness:** the 3-D distance from a query `p` to the circle
/// swept by a profile point `(r_c, z_c)` is `√((rad_p − r_c)² + (z_p − z_c)²)`
/// — minimized at the query's own azimuth whenever `rad_p, r_c ≥ 0` — so the
/// 3-D signed distance of the revolved solid equals the EXACT 2-D polygon
/// signed distance evaluated at `(√(x²+y²), z)`. An exact Euclidean SDF is
/// 1-Lipschitz, hence safe for narrow-band block pruning.
struct RevolvedPolygonSdf {
	pts: Vec<Vec2>,
	bounds: Aabb,
}

impl RevolvedPolygonSdf {
	fn new(pts: Vec<Vec2>) -> Self {
		assert!(pts.len() >= 3, "revolved profile needs >= 3 points");
		let (mut rmax, mut zmin, mut zmax) = (0.0f32, f32::INFINITY, f32::NEG_INFINITY);
		for (i, p) in pts.iter().enumerate() {
			assert!(p.x >= 0.0 && p.is_finite(), "profile radius must be finite and >= 0, got {p:?}");
			assert!(pts[(i + 1) % pts.len()] != *p, "duplicate consecutive profile point {p:?}");
			rmax = rmax.max(p.x);
			zmin = zmin.min(p.y);
			zmax = zmax.max(p.y);
		}
		let bounds = Aabb::new(Vec3::new(-rmax, -rmax, zmin), Vec3::new(rmax, rmax, zmax));
		Self { pts, bounds }
	}

	/// Exact signed distance to the profile polygon (negative inside), winding
	/// agnostic (even-odd crossing sign, Inigo Quilez's `sdPolygon`).
	fn profile_distance(&self, q: Vec2) -> f32 {
		let v = &self.pts;
		let n = v.len();
		let mut d = (q - v[0]).length_squared();
		let mut s = 1.0f32;
		let mut j = n - 1;
		for i in 0..n {
			let e = v[j] - v[i];
			let w = q - v[i];
			let b = w - e * (w.dot(e) / e.dot(e)).clamp(0.0, 1.0);
			d = d.min(b.length_squared());
			let c = [q.y >= v[i].y, q.y < v[j].y, e.x * w.y > e.y * w.x];
			if c == [true; 3] || c == [false; 3] {
				s = -s;
			}
			j = i;
		}
		s * d.sqrt()
	}
}

impl Sdf for RevolvedPolygonSdf {
	fn distance(&self, p: Vec3) -> f32 {
		self.profile_distance(Vec2::new((p.x * p.x + p.y * p.y).sqrt(), p.z))
	}

	fn bounds(&self) -> Aabb {
		self.bounds
	}
}

/// Profile polygon → `RevolvedPolygonSdf` leaf node (f64 profile points).
fn revolved(pts: &[DVec2]) -> Node {
	Node::primitive(RevolvedPolygonSdf::new(pts.iter().map(|p| Vec2::new(p.x as f32, p.y as f32)).collect()))
}

/// Conformal jacket-cavity profile helper: a point at `r_inner(z) + off`.
fn at(z: f64, off: f64) -> DVec2 {
	DVec2::new(r_inner(z) + off, z)
}

/// Cooling-jacket outer shell as one revolved polygon: solid manifold rings at
/// both ends (z 8..13 and 37..42, attached 1 mm INTO the wall) bridged by a
/// 2.5 mm shell that leaves a 5 mm lattice cavity (wall+4 … wall+9, z 13..37).
fn shell_profile() -> Vec<DVec2> {
	let mut pts = vec![at(8.0, 3.0)];
	for z in [8.0, 12.0, 16.0, 20.0, 24.0, 28.0, 32.0, 36.0, 40.0, 42.0] {
		pts.push(at(z, 11.5)); // outer contour, bottom → top
	}
	pts.push(at(42.0, 3.0)); // top attach, buried in the wall
	pts.push(at(37.0, 3.0));
	pts.push(at(37.0, 9.0)); // top-ring underside
	for z in [33.0, 29.0, 25.0, 21.0, 17.0, 13.0] {
		pts.push(at(z, 9.0)); // shell inner contour, top → bottom
	}
	pts.push(at(13.0, 3.0)); // bottom-ring top side, buried
	pts.push(at(10.0, 3.0));
	pts
}

/// Conformal pin-fin truss for the jacket cavity: 6 rings × 20 spokes of
/// tapered radial pins (r 1.2 at the hot wall → 0.9 at the shell — tapered
/// struts are the cone-capsule's native trick) plus X-braces between
/// neighbouring rings. Node radii follow `r_inner(z)`, so the truss is
/// conformal; every strut end is buried 0.8 mm inside wall or shell so the
/// `fillet_union` weld is seamless.
fn jacket_lattice() -> BeamLattice {
	let rings: [f64; 6] = [15.0, 19.0, 23.0, 27.0, 31.0, 35.0];
	let spokes = 20u32;
	let mut nodes = Vec::new();
	let mut struts = Vec::new();
	for (i, &z) in rings.iter().enumerate() {
		for j in 0..spokes {
			let a = (j as f64 + 0.5 * (i % 2) as f64) * TAU / spokes as f64;
			let dir = Vec3::new(a.cos() as f32, a.sin() as f32, 0.0);
			let wall_node = dir * (r_inner(z) + 3.2) as f32 + Vec3::new(0.0, 0.0, z as f32);
			let shell_node = dir * (r_inner(z) + 9.8) as f32 + Vec3::new(0.0, 0.0, z as f32);
			nodes.push(wall_node);
			nodes.push(shell_node);
			let (wa, sa) = ((nodes.len() - 2) as u32, (nodes.len() - 1) as u32);
			struts.push((wa, sa, 1.2, 0.9)); // tapered pin, thick end on the hot wall
			if i > 0 {
				// X-braces to the ring below (same spoke; the half-step ring
				// stagger makes them genuinely diagonal).
				let (wb, sb) = (wa - 2 * spokes, sa - 2 * spokes);
				struts.push((wa, sb, 0.8, 0.8));
				struts.push((wb, sa, 0.8, 0.8));
			}
		}
	}
	BeamLattice::new(nodes, struts)
}

/// The tapered conformal spiral channel through the bell wall (z 5 … 40,
/// 3 turns): the path rides the mid-wall radius `r_inner(z) + 2`, the tube
/// radius pinches to 1.0 mm at the throat (z ≈ 30.5, peak heat flux → highest
/// coolant velocity) and relaxes to 1.4 mm far from it — real regenerative
/// design practice, and a direct exercise of `Pipe`'s per-vertex taper.
fn bell_spiral() -> Pipe {
	let n = 150;
	let mut path = Vec::with_capacity(n + 1);
	let mut radii = Vec::with_capacity(n + 1);
	for i in 0..=n {
		let t = i as f64 / n as f64;
		let z = 5.0 + 35.0 * t;
		let a = 3.0 * TAU * t;
		let r = r_inner(z) + 2.0;
		path.push(Vec3::new((r * a.cos()) as f32, (r * a.sin()) as f32, z as f32));
		radii.push((1.0 + 0.4 * (((z - 30.5).abs() / 15.0).min(1.0))) as f32);
	}
	Pipe::new(path, radii)
}

/// The full rocket part. Returns true when every gate passes.
fn rocket_engine(dir: &str) -> bool {
	// --- EXACT HALF: every machined interface is analytic ------------------------
	// Bell + chamber wall as ONE revolve of the closed wall profile, and the Ø64
	// flange as a bare cylinder whose top rim is filleted FIRST (exact rolling-ball
	// Surface::Torus band) while its cap is still a full disc — the same stable
	// fillet-then-union-then-drill order as hybrid_showcase's lattice_mount. The
	// chamber wall pierces 1 mm through the flange top (a raised seal land), so the
	// union has no coincident faces.
	let bell = revolve(&wall_profile(), 64);
	let flange = cylinder(DVec3::new(0.0, 0.0, 48.0), DVec3::Z, 32.0, 7.0, 64);
	let flange = fillet_circular_rim(&flange, DVec3::new(32.0, 0.0, 55.0), 2.5, 8).expect("flange rim fillets to an exact torus");
	let m = union(&bell, &flange);
	// THEN drill: the Ø34 injector-seat counterbore re-opens the bore through the
	// flange plug, leaving a 1 mm retaining lip over the Ø36 chamber (the offset
	// also keeps the cutter wall clear of the chamber wall — general position),
	// plus an 8 × Ø5 bolt circle at R26.
	let mut m = difference(&m, &cylinder(DVec3::new(0.0, 0.0, 47.0), DVec3::Z, 17.0, 9.5, 64));
	for i in 0..8 {
		let a = i as f64 * PI / 4.0;
		m = difference(&m, &cylinder(DVec3::new(26.0 * a.cos(), 26.0 * a.sin(), 47.0), DVec3::Z, 2.5, 9.0, 24));
	}
	let v = validate(&m);
	let torus_faces = m.faces().filter(|&f| matches!(m.face(f).surface, Surface::Torus { .. })).count();
	let mp = mass_properties(&m);
	let skin = tessellate_adaptive_tol(&m, 0.02);
	let skin_wt = skin.is_watertight();
	skin.write_stl_binary(format!("{dir}/rocket_skin.stl")).expect("write stl");
	std::fs::write(format!("{dir}/rocket_skin.step"), export_step(&m, "rocket_skin")).expect("write step");
	println!(
		"  rocket skin   closed={} manifold={} genus={} (want 9: ring + 8 bolt holes)  {} exact torus faces  vol={:.0} mm³ CoM z={:.2} mm (analytic)  {} tris watertight={}",
		v.closed,
		v.manifold,
		v.genus,
		torus_faces,
		mp.volume,
		mp.center_of_mass.z,
		skin.triangle_count(),
		skin_wt
	);

	// --- VOXEL HALF: closed-form implicit twin + the W3 primitives ----------------
	// The twin mirrors the exact half with ns-per-query closed forms: the SAME wall
	// profile revolved (RevolvedPolygonSdf is the exact SDF of the revolve's
	// analytic intent), the flange ring with its rim corner chord-cut and the EXACT
	// torus unioned over it (the chord lies strictly inside the tube, so polygon ∪
	// torus IS the filleted profile), and the 8 bolt drills.
	let flange_ring = || {
		revolved(&[
			DVec2::new(17.0, 48.0),
			DVec2::new(32.0, 48.0),
			DVec2::new(32.0, 52.5),
			DVec2::new(29.5, 55.0),
			DVec2::new(17.0, 55.0),
		])
		.union(Node::primitive(VoxTorus::new(Vec3::new(0.0, 0.0, 52.5), Vec3::Z, 29.5, 2.5)))
	};
	let bolts = || {
		(0..8).fold(None::<Node>, |acc, i| {
			let a = i as f32 * std::f32::consts::PI / 4.0;
			let hole = Node::primitive(VoxCylinder::new(
				Vec3::new(26.0 * a.cos(), 26.0 * a.sin(), 47.0),
				Vec3::new(26.0 * a.cos(), 26.0 * a.sin(), 56.0),
				2.5,
			));
			Some(match acc {
				Some(n) => n.union(hole),
				None => hole,
			})
		})
		.expect("eight bolt holes")
	};
	let structure = || revolved(&wall_profile()).union(flange_ring()).union(revolved(&shell_profile())).difference(bolts());

	let lat = jacket_lattice();
	let (lat_nodes, lat_struts, lat_vol) = (lat.node_count(), lat.strut_count(), lat.strut_volume_estimate());
	let spiral = bell_spiral();
	let helix = Pipe::helix(Vec3::new(0.0, 0.0, 44.5), Vec3::Z, 20.0, 4.0, 1.9, 64, 1.3);
	let channel_est = spiral.volume_estimate() + helix.volume_estimate();
	let (spiral_segs, helix_segs) = (spiral.segment_count(), helix.segment_count());
	let channels = Node::primitive(spiral).union(Node::primitive(helix));

	// FUSION: weld the pin-fin truss onto wall + shell with a radius-1.2 SDF
	// fillet, then carve the channels. Both pipes stay ≥ 0.6 mm under every
	// surface (4 mm wall, Ø2.0–2.8 channels mid-wall), so carving cannot open them.
	let carved = fillet_union(structure(), Node::primitive(jacket_lattice()), 1.2).difference(channels);
	let solid_twin = fillet_union(structure(), Node::primitive(lat), 1.2); // channel-less reference

	// Extraction at resin resolution with MANIFOLD dual contouring: the truss weld
	// grazes the shells tangentially in many places, and only the manifold mesher
	// resolves those pinch saddles. A residual grid pinch can survive any fixed
	// grid: snip it apart (geometry-preserving); if cracks remain, re-extract on a
	// slightly shifted grid — cheap, the twin is pure closed-form SDF.
	let bounds = Aabb::from_center_half_extent(Vec3::new(0.0, 0.0, 28.0), Vec3::new(33.5, 33.5, 28.8));
	let mut hmesh = manifold_dual_contour(&carved, bounds, Resolution::VoxelSize(VOXEL));
	if check_mesh(&hmesh).non_manifold_edges > 0 || !hmesh.is_watertight() {
		hmesh = make_manifold(&hmesh);
		if check_mesh(&hmesh).non_manifold_edges > 0 || !hmesh.is_watertight() {
			hmesh = manifold_dual_contour(&carved, bounds.pad(VOXEL), Resolution::VoxelSize(VOXEL * 1.07));
		}
	}
	let hr = check_mesh(&hmesh);
	let hvol = hmesh.signed_volume().abs();
	hmesh.write_stl_binary(format!("{dir}/rocket_hybrid.stl")).expect("write stl");
	hmesh.write_3mf(format!("{dir}/rocket_hybrid.3mf")).expect("write 3mf");
	println!(
		"  rocket hybrid {} tris  watertight={} non-manifold={}  vol {:.0} mm³ (skin {:.0} + jacket − channels)  truss {} nodes / {} struts (≈{:.0} mm³)",
		hmesh.triangle_count(),
		hmesh.is_watertight(),
		hr.non_manifold_edges,
		hvol,
		mp.volume,
		lat_nodes,
		lat_struts,
		lat_vol
	);

	// HOLLOWNESS BY PHYSICS: extract the channel-less twin and the carved twin on
	// the IDENTICAL narrow-band grid — away from the channels the two fields are
	// bit-equal, so the outer-surface quantization cancels and the volume
	// difference isolates the carved material, which must equal the pipes'
	// analytic tube volume within the documented 5 % (`Pipe::volume_estimate`).
	// Narrow-band DC is the fast closed-volume route: truss junction saddles may
	// pinch (nme > 0, honest lattice limitation), but the surface stays CLOSED
	// (boundary edges = 0), which is all a signed volume needs.
	let vref_mesh = dual_contour_narrowband(&solid_twin, bounds, Resolution::VoxelSize(VERIFY_VOXEL));
	let vcar_mesh = dual_contour_narrowband(&carved, bounds, Resolution::VoxelSize(VERIFY_VOXEL));
	let (rref, rcar) = (check_mesh(&vref_mesh), check_mesh(&vcar_mesh));
	let removed = vref_mesh.signed_volume() - vcar_mesh.signed_volume();
	let closed_pair = rref.boundary_edges == 0 && rcar.boundary_edges == 0;
	let hollow_ok = closed_pair && (removed - channel_est).abs() / channel_est < 0.05;
	println!(
		"  cooling       spiral {} segs (taper Ø2.0 throat → Ø2.8) + helix {} segs  carved {removed:.0} mm³ vs analytic {channel_est:.0} mm³ ({:+.1}%)  verify pair closed={closed_pair}",
		spiral_segs,
		helix_segs,
		(removed / channel_est - 1.0) * 100.0
	);

	let ok = v.closed
		&& v.manifold
		&& v.genus == 9
		&& torus_faces > 0
		&& skin_wt
		&& hmesh.is_watertight()
		&& hr.non_manifold_edges == 0
		&& hvol > mp.volume
		&& hollow_ok;
	println!("  rocket_engine {}", if ok { "PASS" } else { "FAIL" });
	ok
}

fn main() {
	let dir = "rocket_out";
	std::fs::create_dir_all(dir).expect("create output dir");
	println!("Rocket thrust chamber — exact B-rep skin + W3 voxel primitives (Pipe, BeamLattice):\n");
	let ok = rocket_engine(dir);
	println!("\n{} — wrote files to ./{dir}/", if ok { "ALL PASS" } else { "FAILED" });
	std::process::exit(if ok { 0 } else { 1 });
}
