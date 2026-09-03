//! TPMS LEAF CATCHER — a gyroid pre-filter basket for pool skimmers and inlet
//! buckets (Printables "Pool Accessories" flash contest, July 2026 — entry 3,
//! the implicit-modeling showcase).
//!
//! An open-top Ø98 × 80 basket whose side wall and floor are a GYROID SHEET
//! lattice (cell 13 mm, sheet half-thickness 1.45 mm): a self-supporting
//! double-labyrinth wall with huge open area — water pours through, leaves,
//! bugs and seed pods stay in. A solid rim ring (8 mm tall) with a Ø98 hang
//! flange caps the lattice; the basket hangs by its flange in any Ø93–Ø96
//! round opening (skimmer throat, bucket lid, or the printed `hanger_ring_93`
//! adapter, which is built on the EXACT B-rep side — `extrude_with_holes`,
//! `validate`, analytic `volume` — making this a true hybrid part set).
//!
//! Route (honest): the lattice is closed-form implicit geometry — a `Tpms`
//! sheet (wrapped `Node::primitive_bound`, since the normalized trig field is
//! a distance BOUND, not an exact distance) intersected with the cup region
//! and unioned with the solid rim and six stiffening ribs, all exact-interior
//! cylinder/cuboid SDFs. It is meshed by Manifold Dual Contouring — the DENSE
//! mesher, which per the FieldQuality contract needs no Lipschitz bound
//! (narrow-band would); the field's ≤1-Lipschitz claim is still verified below
//! as a gate, by sampled PAIRWISE ratios |f(p)−f(q)|/|p−q| (the actual
//! Lipschitz property — a finite-difference gradient would over-read √2 at the
//! kinks/medial ridges every exact min/max field has). Sheet junctions can
//! pinch at MDC cell corners: if the raw extraction has non-manifold edges it
//! is healed ONCE by `make_manifold` (edge separation; volume-preserving,
//! never worse than its input) and the heal is REPORTED in the table, never
//! silent — connectivity (one shell, no floating lattice debris) is gated on
//! the RAW mesh, manifoldness on the healed one.
//!
//! SUPPORT HONESTY: a gyroid is NOT support-free by the strict 45° metric —
//! ~8 % of the basket's area faces down steeper than 45° (measured and gated
//! below against a pinned budget, not asserted to zero). Those overhangs are
//! short local arcs (≤ half a 13 mm cell) that FDM prints cleanly without
//! support — the same reason gyroid infill works — and DESIGN.md says so
//! plainly. The flat hanger ring IS support-free and gated strictly (steep
//! < 1e-6). A wrong-orientation negative control proves the support gate
//! bites, a solid-region control proves the solid-fraction band bites, and a
//! thin-sheet control proves the wall-thickness gate bites.
//!
//! Contract: pool_system/tpms_basket/DESIGN.md (every line asserted here).
//! Run: cargo run --example pool_tpms_basket -p kernel-model --release
//!   -> pool_system/tpms_basket/ (exit 1 on any FAIL)

use kernel_brep::math::DVec2;
use kernel_brep::{extrude_with_holes, tessellate_default, validate, volume, Solid};
use kernel_core::math::Vec3;
use kernel_core::mesh::Mesh;
use kernel_core::{check_mesh, make_manifold};
use kernel_implicit::{manifold_dual_contour, Aabb, Cuboid, Cylinder, Node, Resolution, Sdf, Tpms, TpmsKind};
use std::f32::consts::TAU;

// ---- basket envelope (mm) ------------------------------------------------------
const R_OUT: f32 = 45.0; // lattice wall outer radius (Ø90 body)
const R_IN: f32 = 39.0; // lattice wall inner radius (6 mm lattice band)
const H: f32 = 80.0; // overall height
const FLOOR_T: f32 = 6.0; // lattice floor thickness
const RIM_Z0: f32 = 72.0; // solid rim ring: z ∈ [72, 80]
const FLANGE_R: f32 = 49.0; // hang flange outer radius (Ø98)
const FLANGE_Z0: f32 = 77.0; // flange: z ∈ [77, 80]
const LAT_TOP: f32 = 73.5; // lattice clipped 1.5 INTO the rim (overlap, not kiss)
const CAV_Z0: f32 = FLOOR_T; // cavity floor = top of the lattice floor

// ---- the gyroid sheet ----------------------------------------------------------
const CELL: f32 = 13.0; // unit-cell period (pore channels ~ cell/2 - wall ≈ 4 mm)
/// Sheet half-thickness parameter. The `Tpms` field is ≤1-Lipschitz (trig field
/// divided by its pinned gradient bound √3 for the gyroid), so every medial-
/// surface point is ≥ SHEET_HALF/√3 from the sheet boundary — a GUARANTEED wall
/// floor of 2·1.45/√3 = 1.674 mm ≥ the 1.6 mm printability floor (two 0.4 mm
/// perimeters × 2 walls). Typical walls are thicker (the bound is attained only
/// where the trig gradient peaks); both floor and typical are MEASURED below by
/// marching the field, not just claimed.
const SHEET_HALF: f32 = 1.45;
const WALL_FLOOR: f64 = 1.6; // printability floor gate on measured min thickness

// ---- stiffening ribs + hanger ring + assembly context --------------------------
const N_RIBS: usize = 6; // solid vertical ribs bed→rim (stiffness + guaranteed
                         // lattice-to-rim structural continuity)
const RIB_HALF_W: f32 = 1.2; // rib half-thickness (2.4 mm wide)
const RING_BORE_R: f64 = 46.5; // hanger ring bore Ø93 (basket wall Ø90 + 1.5/side)
const RING_OUT_R: f64 = 60.0; // hanger ring outer Ø120
const RING_H: f64 = 6.0;
const THROAT_R_IN: f64 = 50.0; // scene context: Ø100 skimmer-throat bore
const THROAT_R_OUT: f64 = 55.0;
const THROAT_H: f64 = 40.0;

/// MDC cell for the basket. 0.4 mm (the FDM-production anchor, DESIGN_GUIDE
/// §17.4) yields a 1.6 M-triangle / 80 MB STL — right at the campaign's size
/// cap — so the basket ships at 0.5 mm: still ≥ 3.3 cells across the 1.674 mm
/// minimum wall (the guide's ≥3 floor) and QEF chord error far below a 0.2 mm
/// layer line. The coupon ships at 0.42: at exactly 0.4 its extraction leaves
/// two bowtie pinch VERTICES the edge-separation heal declines (the heal is
/// only accepted when it makes the mesh strictly better), and the junction
/// sliver count is grid-PHASE sensitive — measured 0.08 %–6.6 % of triangles
/// across voxels 0.32–0.5 on this tile. 0.42 extracts a clean 2-manifold with
/// 0.37 % slivers, inside the production range and under the shared budget;
/// the sensitivity itself is documented in DESIGN.md, not hidden.
const VOXEL_BASKET: f32 = 0.5;
const VOXEL_COUPON: f32 = 0.42;
const STL_CAP_BYTES: u64 = 80_000_000;
const SEG: usize = 128;
const PETG_G_PER_MM3: f64 = 0.00127;

// ---- support budgets (measured on this exact geometry, then pinned) ------------
/// Gyroid steep area, basket upright: measured 7310 mm² = 7.9 % of total area —
/// short (≤ half-cell) local overhang arcs, printable unsupported (see module
/// doc + DESIGN.md). Budget pinned ~10 % above measured; a geometry change that
/// grows the unprintable overhang share trips it. The negative control below
/// shows the SAME budget failing the basket in a wrong orientation.
const BASKET_STEEP_BUDGET: f64 = 8000.0;
/// Same metric for the coupon tile: measured 728 mm² (6.4 % of its area).
const COUPON_STEEP_BUDGET: f64 = 820.0;
/// MDC places each junction vertex from its own cell's QEF, so near-tangent
/// sheet junctions leave sliver-scale self-overlaps (no holes — the mesh stays
/// a closed 2-manifold; slicers resolve them by union fill). Measured 0.49 % of
/// triangles on the basket, 0.37 % on the coupon, 0 on the B-rep ring; gated
/// at ≤ 1 %.
const SELF_X_FRAC_BUDGET: f64 = 0.01;

// ---- small deterministic RNG (no dep; reproducible sampling) --------------------
struct Lcg(u64);
impl Lcg {
	fn next_f32(&mut self) -> f32 {
		self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
		((self.0 >> 33) as f32) / (u32::MAX >> 1) as f32
	}
}

// ---- implicit model -------------------------------------------------------------

fn big_region() -> Aabb {
	Aabb::new(Vec3::splat(-200.0), Vec3::splat(200.0))
}

fn zcyl(z0: f32, z1: f32, r: f32) -> Node {
	Node::primitive(Cylinder::new(Vec3::new(0.0, 0.0, z0), Vec3::new(0.0, 0.0, z1), r))
}

/// The gyroid sheet clipped to the cup region (side wall band + floor disc).
fn lattice_node() -> Node {
	let sheet = Tpms::sheet(big_region(), TpmsKind::Gyroid, CELL, SHEET_HALF);
	let cup = zcyl(0.0, LAT_TOP, R_OUT).difference(zcyl(CAV_Z0, LAT_TOP + 1.0, R_IN));
	Node::primitive_bound(sheet).intersection(cup)
}

/// Solid rim ring + hang flange (exact-interior cylinder SDFs).
fn rim_node() -> Node {
	zcyl(RIM_Z0, H, R_OUT)
		.union(zcyl(FLANGE_Z0, H, FLANGE_R))
		.difference(zcyl(RIM_Z0 - 1.0, H + 1.0, R_IN))
}

/// Six solid vertical ribs, bed to rim, trimmed to the outer cylinder. Each rib
/// pokes 1 mm past R_IN into the cavity so it overlaps the full lattice band.
fn ribs_node() -> Node {
	let y0 = R_IN - 1.0;
	let y1 = R_OUT + 0.5;
	let rib = Node::primitive(Cuboid::new(
		Vec3::new(0.0, (y0 + y1) / 2.0, (RIM_Z0 + 4.0) / 2.0),
		Vec3::new(RIB_HALF_W, (y1 - y0) / 2.0, (RIM_Z0 + 4.0) / 2.0),
	));
	rib.circular_pattern(Vec3::ZERO, Vec3::Z, TAU / N_RIBS as f32, N_RIBS)
		.intersection(zcyl(-1.0, RIM_Z0 + 5.0, R_OUT))
}

fn basket_node() -> Node {
	lattice_node().union(rim_node()).union(ribs_node())
}

/// Printability coupon: 45×45×10 tile — 3 mm solid frame + the SAME gyroid
/// sheet (cell/thickness identical to the basket). Print this first.
fn coupon_node() -> Node {
	let outer = Node::primitive(Cuboid::from_corners(Vec3::new(-22.5, -22.5, 0.0), Vec3::new(22.5, 22.5, 10.0)));
	let inner = Node::primitive(Cuboid::from_corners(Vec3::new(-19.5, -19.5, -1.0), Vec3::new(19.5, 19.5, 11.0)));
	let frame = outer.difference(inner);
	let region = Node::primitive(Cuboid::from_corners(Vec3::new(-20.5, -20.5, 0.0), Vec3::new(20.5, 20.5, 10.0)));
	let sheet = Tpms::sheet(big_region(), TpmsKind::Gyroid, CELL, SHEET_HALF);
	frame.union(Node::primitive_bound(sheet).intersection(region))
}

// ---- mesh helpers ---------------------------------------------------------------

fn mesh_aabb(m: &Mesh) -> (Vec3, Vec3) {
	let mut lo = Vec3::splat(f32::INFINITY);
	let mut hi = Vec3::splat(f32::NEG_INFINITY);
	for p in &m.positions {
		lo = lo.min(*p);
		hi = hi.max(*p);
	}
	(lo, hi)
}

fn drop_to_bed(m: &mut Mesh) {
	let zmin = m.positions.iter().map(|p| p.z).fold(f32::INFINITY, f32::min);
	for p in &mut m.positions {
		p.z -= zmin;
	}
}

/// Rotate a mesh -90° about X (Z-up becomes Y-up): the basket on its side.
fn on_side(m: &Mesh) -> Mesh {
	let mut r = m.clone();
	for p in &mut r.positions {
		*p = Vec3::new(p.x, p.z, -p.y);
	}
	drop_to_bed(&mut r);
	r
}

fn translated(m: &Mesh, dz: f32) -> Mesh {
	let mut p = m.clone();
	for q in &mut p.positions {
		q.z += dz;
	}
	p
}

fn merge_into(dst: &mut Mesh, src: &Mesh) {
	let base = dst.positions.len() as u32;
	dst.positions.extend_from_slice(&src.positions);
	dst.indices.extend(src.indices.iter().map(|i| i + base));
}

/// Number of vertex-connected shells — 1 means no floating lattice debris
/// (a disconnected sheet island would print as loose junk in the wall).
fn shell_count(m: &Mesh) -> usize {
	let n = m.positions.len();
	let mut parent: Vec<u32> = (0..n as u32).collect();
	fn find(parent: &mut [u32], mut x: u32) -> u32 {
		while parent[x as usize] != x {
			parent[x as usize] = parent[parent[x as usize] as usize];
			x = parent[x as usize];
		}
		x
	}
	for t in m.indices.chunks_exact(3) {
		let a = find(&mut parent, t[0]);
		for &v in &t[1..] {
			let b = find(&mut parent, v);
			parent[b as usize] = a;
		}
	}
	(0..n as u32).filter(|&i| find(&mut parent, i) == i).count()
}

// ---- gates ----------------------------------------------------------------------

/// Per-part gate. Connectivity (ONE shell) is judged on the raw mesh; if the
/// raw extraction is not a closed 2-manifold it is healed once by
/// `make_manifold` and the heal is printed, never silent. Then: closed
/// 2-manifold, expected envelope, bed fit, self-intersection fraction below
/// budget, support report vs an honest per-part steep budget, bridges ≤ 12 mm,
/// STL under the campaign size cap. Returns the shipped (healed, bed-dropped)
/// mesh for posing.
fn emit(name: &str, mesh: &Mesh, expect: (f32, f32, f32), steep_budget: f64, ok_all: &mut bool) -> Mesh {
	let shells = shell_count(mesh);
	let healed = !mesh.is_two_manifold();
	let mut m = if healed { make_manifold(mesh) } else { mesh.clone() };
	drop_to_bed(&mut m);
	let rep = check_mesh(&m);
	let selfx_frac = rep.self_intersections as f64 / m.triangle_count() as f64;
	let (lo, hi) = mesh_aabb(&m);
	let ext = hi - lo;
	let dims_ok = (ext.x - expect.0).abs() <= 0.6 && (ext.y - expect.1).abs() <= 0.6 && (ext.z - expect.2).abs() <= 0.6;
	let fits = ext.x <= 250.0 && ext.y <= 210.0 && ext.z <= 220.0;
	let sup = m.support_free_report(Vec3::Z, 45.0, 0.3);
	let vol = m.signed_volume().abs();
	let stl = m.to_stl_binary();
	let ok = rep.watertight
		&& shells == 1
		&& selfx_frac <= SELF_X_FRAC_BUDGET
		&& dims_ok
		&& fits
		&& sup.steep_area <= steep_budget
		&& sup.max_bridge_span <= 12.0
		&& sup.bed_area >= 150.0
		&& (stl.len() as u64) < STL_CAP_BYTES;
	*ok_all &= ok;
	let _ = std::fs::write(format!("pool_system/tpms_basket/parts/{name}.stl"), &stl);
	println!(
		"  {name:22} manifold={:5} healed={healed:5} shells={shells}  selfx={:5}({:4.2}%)  {:5.1}x{:5.1}x{:4.1}mm  steep={:6.1}mm² ({:4.1}%) ≤{steep_budget:6.0}  bridge≤{:4.1}  bed={:5.0}mm²  {:4.0}g  {:4.1}MB  {}",
		rep.watertight,
		rep.self_intersections,
		100.0 * selfx_frac,
		ext.x,
		ext.y,
		ext.z,
		sup.steep_area,
		100.0 * sup.steep_area / sup.total_area,
		sup.max_bridge_span,
		sup.bed_area,
		vol * PETG_G_PER_MM3,
		stl.len() as f64 / 1e6,
		if ok { "OK" } else { "<<< FAIL" }
	);
	m
}

/// Check one designed relation between two posed meshes (POOLDOCK-style).
fn relation(label: &str, a: &Mesh, b: &Mesh, contact: bool, ok: &mut bool) {
	let d = a.min_distance(b);
	let pass = if contact { d < 0.06 } else { d >= 0.10 };
	*ok &= pass;
	println!(
		"  {label:52} min_dist={d:7.3}  want {}  {}",
		if contact { "contact (<0.06)" } else { "clearance (>=0.10)" },
		if pass { "OK" } else { "<<< FAIL" }
	);
}

/// Fraction of grid samples inside `sdf` within the annular band
/// r ∈ [r0, r1], z ∈ [z0, z1] (r0 = 0 samples the full disc). The 0.7003 step
/// with a half-step offset keeps sample planes off the model's exact z-planes.
fn band_fraction(sdf: &dyn Sdf, r0: f32, r1: f32, z0: f32, z1: f32) -> (f64, u64) {
	let step = 0.7003_f32;
	let (mut inside, mut total) = (0u64, 0u64);
	let n_xy = (2.0 * r1 / step).ceil() as i32;
	let n_z = ((z1 - z0) / step).ceil() as i32;
	for k in 0..n_z {
		let z = z0 + (k as f32 + 0.5) * step;
		if z >= z1 {
			continue;
		}
		for i in -n_xy..=n_xy {
			for j in -n_xy..=n_xy {
				let (x, y) = ((i as f32 + 0.5) * step, (j as f32 + 0.5) * step);
				let r = (x * x + y * y).sqrt();
				if r < r0 || r > r1 {
					continue;
				}
				total += 1;
				if sdf.distance(Vec3::new(x, y, z)) < 0.0 {
					inside += 1;
				}
			}
		}
	}
	(inside as f64 / total as f64, total)
}

/// Measure the ACTUAL local sheet thickness of the (unclipped) gyroid sheet:
/// project random seeds onto the medial surface (Newton on the level-0 network
/// field), then march ± along the field gradient to the sheet boundary and
/// bisect the crossings. Returns (min, mean, max, n_measured).
fn measure_sheet_thickness(half: f32, seeds: usize) -> (f64, f64, f64, usize) {
	let sheet = Tpms::sheet(big_region(), TpmsKind::Gyroid, CELL, half);
	let net = Tpms::network(big_region(), TpmsKind::Gyroid, CELL, 0.0);
	let h = 0.02_f32;
	let grad = |p: Vec3| {
		Vec3::new(
			net.distance(p + Vec3::X * h) - net.distance(p - Vec3::X * h),
			net.distance(p + Vec3::Y * h) - net.distance(p - Vec3::Y * h),
			net.distance(p + Vec3::Z * h) - net.distance(p - Vec3::Z * h),
		) / (2.0 * h)
	};
	// One-sided crossing of the sheet boundary from a medial point along `dir`.
	let march = |m: Vec3, dir: Vec3| -> f64 {
		let (mut t_in, mut t_out) = (0.0_f32, 0.0_f32);
		let mut t = 0.05_f32;
		while t <= 4.0 {
			if sheet.distance(m + dir * t) >= 0.0 {
				t_out = t;
				break;
			}
			t_in = t;
			t += 0.05;
		}
		if t_out == 0.0 {
			return 4.0; // wall thicker than the 4 mm cap — plenty; caps never set the min
		}
		for _ in 0..30 {
			let mid = 0.5 * (t_in + t_out);
			if sheet.distance(m + dir * mid) >= 0.0 {
				t_out = mid;
			} else {
				t_in = mid;
			}
		}
		(0.5 * (t_in + t_out)) as f64
	};
	let mut rng = Lcg(0x5eed_cafe);
	let (mut mn, mut mx, mut sum, mut n) = (f64::INFINITY, 0.0_f64, 0.0_f64, 0usize);
	for _ in 0..seeds {
		let ang = rng.next_f32() * TAU;
		let r = R_IN + 0.5 + rng.next_f32() * (R_OUT - R_IN - 1.0);
		let mut p = Vec3::new(r * ang.cos(), r * ang.sin(), 5.0 + rng.next_f32() * 65.0);
		let mut converged = false;
		for _ in 0..15 {
			let s = net.distance(p);
			if s.abs() < 1e-4 {
				converged = true;
				break;
			}
			let g = grad(p);
			if g.length_squared() < 1e-8 {
				break;
			}
			p -= g * (s / g.length_squared());
		}
		if !converged {
			continue;
		}
		let dir = grad(p).normalize();
		let t = march(p, dir) + march(p, -dir);
		mn = mn.min(t);
		mx = mx.max(t);
		sum += t;
		n += 1;
	}
	(mn, sum / n as f64, mx, n)
}

/// Grid-integrated SDF volume over the basket domain (offset grid, mm³).
fn sdf_volume(sdf: &dyn Sdf) -> f64 {
	let step = 0.7003_f32;
	let mut inside = 0u64;
	let n = (101.0 / step).ceil() as i32;
	let n_z = (81.5 / step).ceil() as i32;
	for k in 0..n_z {
		let z = -0.5 + (k as f32 + 0.5) * step;
		for i in -n..=n {
			for j in -n..=n {
				let p = Vec3::new((i as f32 + 0.5) * step, (j as f32 + 0.5) * step, z);
				if sdf.distance(p) < 0.0 {
					inside += 1;
				}
			}
		}
	}
	inside as f64 * (step as f64).powi(3)
}

fn circle(r: f64) -> Vec<DVec2> {
	(0..SEG)
		.map(|i| {
			let a = std::f64::consts::TAU * i as f64 / SEG as f64;
			DVec2::new(r * a.cos(), r * a.sin())
		})
		.collect()
}

/// Exact-B-rep annular ring (bore `r_in`, outer `r_out`, height `h`), validated
/// and checked against the analytic annulus volume (only the 128-gon chord
/// deficit, ~0.04 %, separates engine volume from πr² arithmetic).
fn brep_ring(r_in: f64, r_out: f64, h: f64, label: &str, ok: &mut bool) -> (Solid, Mesh) {
	let s = extrude_with_holes(&circle(r_out), &[circle(r_in)], h);
	let val = validate(&s);
	let v = volume(&s);
	let v_ref = std::f64::consts::PI * (r_out * r_out - r_in * r_in) * h;
	let pass = val.is_valid() && ((v - v_ref) / v_ref).abs() < 2e-3;
	*ok &= pass;
	println!(
		"  {label:22} brep valid={:5}  volume {v:9.0} vs analytic {v_ref:9.0} mm³  {}",
		val.is_valid(),
		if pass { "OK" } else { "<<< FAIL" }
	);
	let m = tessellate_default(&s);
	(s, m)
}

fn main() {
	let _ = std::fs::create_dir_all("pool_system/tpms_basket/parts");
	let _ = std::fs::create_dir_all("pool_system/tpms_basket/assembly_parts");
	println!("TPMS LEAF CATCHER — gyroid pre-filter basket (STLs in print orientation):\n");
	let mut ok = true;

	// ---- field honesty: the assembled basket field must keep the ≤1-Lipschitz
	// claim its Tpms leaves make (union/intersection of ≤1-Lipschitz fields stays
	// ≤1-Lipschitz; the dense MDC mesher does not NEED the bound — this checks the
	// claim anyway). Verified on sampled point PAIRS, the property itself.
	let basket = basket_node();
	let mut rng = Lcg(0x0bad_5eed);
	let mut max_ratio = 0.0_f32;
	for _ in 0..40_000 {
		let p = Vec3::new(
			rng.next_f32() * 100.0 - 50.0,
			rng.next_f32() * 100.0 - 50.0,
			rng.next_f32() * 82.0 - 1.0,
		);
		let dir = (Vec3::new(rng.next_f32(), rng.next_f32(), rng.next_f32()) - Vec3::splat(0.5)).normalize_or_zero();
		if dir == Vec3::ZERO {
			continue;
		}
		let step = 0.05 + rng.next_f32() * 2.0;
		let q = p + dir * step;
		max_ratio = max_ratio.max((basket.distance(p) - basket.distance(q)).abs() / step);
	}
	let lip_ok = max_ratio <= 1.001;
	ok &= lip_ok;
	println!(
		"  field contract: sampled max |f(p)-f(q)|/|p-q| = {max_ratio:.4} (claim ≤ 1, tol 1.001) {}\n",
		if lip_ok { "OK" } else { "<<< FAIL" }
	);

	// ---- mesh the implicit parts (dense Manifold DC — no Lipschitz needed) ----
	let dom_basket = Aabb::new(Vec3::new(-50.2, -50.2, -0.8), Vec3::new(50.2, 50.2, 80.8));
	let m_basket_raw = manifold_dual_contour(&basket, dom_basket, Resolution::VoxelSize(VOXEL_BASKET));
	let dom_coupon = Aabb::new(Vec3::new(-23.5, -23.5, -0.8), Vec3::new(23.5, 23.5, 10.8));
	let m_coupon_raw = manifold_dual_contour(&coupon_node(), dom_coupon, Resolution::VoxelSize(VOXEL_COUPON));

	// ---- per-part gates --------------------------------------------------------
	let m_basket = emit("basket_leafcatcher_90", &m_basket_raw, (2.0 * FLANGE_R, 2.0 * FLANGE_R, H), BASKET_STEEP_BUDGET, &mut ok);
	// the hanger ring is EXACT B-rep (the hybrid's other half)
	let (_ring, m_ring_raw) = brep_ring(RING_BORE_R, RING_OUT_R, RING_H, "hanger_ring_93", &mut ok);
	let m_ring = emit("hanger_ring_93", &m_ring_raw, (120.0, 120.0, 6.0), 1e-6, &mut ok);
	emit("coupon_gyroid_45", &m_coupon_raw, (45.0, 45.0, 10.0), COUPON_STEEP_BUDGET, &mut ok);

	// ---- lattice solid fraction (SDF sampling, the design band) ----------------
	println!("\nlattice / solid-region fields (grid-sampled on the SDF):");
	let lattice = lattice_node();
	let (f_wall, n_wall) = band_fraction(&lattice, R_IN + 0.5, R_OUT - 0.5, 8.0, 64.0);
	let (f_floor, n_floor) = band_fraction(&lattice, 0.0, R_IN - 1.0, 1.0, 5.0);
	let wall_ok = (0.30..=0.60).contains(&f_wall);
	let floor_ok = (0.30..=0.60).contains(&f_floor);
	ok &= wall_ok && floor_ok;
	println!("  wall band  solid fraction {f_wall:.3} of {n_wall} pts (want 0.30–0.60) {}", if wall_ok { "OK" } else { "<<< FAIL" });
	println!("  floor band solid fraction {f_floor:.3} of {n_floor} pts (want 0.30–0.60) {}", if floor_ok { "OK" } else { "<<< FAIL" });

	// rim + flange must be FULLY solid: every sample in the 0.7 mm-shrunk region
	// is strictly inside the field.
	let (f_rim, n_rim) = band_fraction(&basket, R_IN + 0.7, R_OUT - 0.7, RIM_Z0 + 0.7, H - 0.7);
	let (f_flange, n_flange) = band_fraction(&basket, R_OUT + 0.7, FLANGE_R - 0.7, FLANGE_Z0 + 0.7, H - 0.7);
	let rim_solid = f_rim == 1.0 && f_flange == 1.0;
	ok &= rim_solid;
	println!(
		"  rim ring  solid fraction {f_rim:.3} of {n_rim} pts, flange {f_flange:.3} of {n_flange} (want 1.000) {}",
		if rim_solid { "OK" } else { "<<< FAIL" }
	);

	// mesh volume must agree with the field it was extracted from (ties the STL
	// to the SDF: no silent extraction loss; the heal preserves volume). Grid
	// integration is honest to ~1 %.
	let v_mesh = m_basket.signed_volume().abs();
	let v_sdf = sdf_volume(&basket);
	let v_err = (v_mesh - v_sdf).abs() / v_sdf;
	let v_ok = v_err < 0.02;
	ok &= v_ok;
	println!("  basket volume: mesh {v_mesh:.0} vs SDF-integrated {v_sdf:.0} mm³ (Δ {:.2}%, want <2%) {}", 100.0 * v_err, if v_ok { "OK" } else { "<<< FAIL" });

	// ---- wall thickness: guaranteed floor (Lipschitz arithmetic) + MEASURED ----
	println!("\nsheet wall thickness (marched on the field, 400 medial seeds):");
	let floor_guaranteed = 2.0 * SHEET_HALF as f64 / 3.0_f64.sqrt();
	let (t_min, t_mean, t_max, n_t) = measure_sheet_thickness(SHEET_HALF, 400);
	// measured min may sit marginally under the analytic floor only by the 0.05 mm
	// marching/projection tolerance — anything more would falsify the Lipschitz claim.
	let t_ok = floor_guaranteed >= WALL_FLOOR && t_min >= WALL_FLOOR && t_min >= floor_guaranteed - 0.05;
	ok &= t_ok;
	println!(
		"  guaranteed ≥ {floor_guaranteed:.3} (2·{SHEET_HALF}/√3); measured min {t_min:.3} / mean {t_mean:.3} / max {t_max:.3} over {n_t} pts (floor {WALL_FLOOR}) {}",
		if t_ok { "OK" } else { "<<< FAIL" }
	);

	// ---- negative controls: every non-trivial gate must BITE -------------------
	println!("\nnegative controls:");
	// (1) support gate: the basket on its side turns the wall into an arch — the
	// SAME steep budget that passes upright must fail there.
	let side_rep = on_side(&m_basket).support_free_report(Vec3::Z, 45.0, 0.3);
	let nc1 = side_rep.steep_area > BASKET_STEEP_BUDGET;
	ok &= nc1;
	println!(
		"  basket audited on its side: steep {:.0} mm² (must exceed the {:.0} budget) {}",
		side_rep.steep_area,
		BASKET_STEEP_BUDGET,
		if nc1 { "OK" } else { "<<< FAIL" }
	);
	// (2) fraction band: a SOLID region sampled the same way must land outside
	// 0.30–0.60 (the band genuinely discriminates lattice from solid).
	let nc2 = !(0.30..=0.60).contains(&f_rim);
	ok &= nc2;
	println!("  solid rim vs lattice band: fraction {f_rim:.3} outside 0.30–0.60 {}", if nc2 { "OK" } else { "<<< FAIL" });
	// (3) thickness gate: a 0.5 mm-half sheet must measure UNDER the 1.6 floor.
	let (thin_min, _, _, _) = measure_sheet_thickness(0.5, 200);
	let nc3 = thin_min < WALL_FLOOR;
	ok &= nc3;
	println!("  thin-sheet control (half 0.5): measured min {thin_min:.3} < {WALL_FLOOR} {}", if nc3 { "OK" } else { "<<< FAIL" });

	// ---- posed fits + assembly scene -------------------------------------------
	// Scene: a Ø100/Ø110 skimmer-throat stub (context, exact B-rep); the hanger
	// ring rests on its top face; the basket hangs through the ring by its
	// flange. All three shipped as posed component STLs for the assembly doc.
	println!("\nposed fits (throat bore Ø{:.0}, ring bore Ø{:.0}, basket wall Ø{:.0}, flange Ø{:.0}):", 2.0 * THROAT_R_IN, 2.0 * RING_BORE_R, 2.0 * R_OUT, 2.0 * FLANGE_R);
	let (_throat, m_throat_raw) = brep_ring(THROAT_R_IN, THROAT_R_OUT, THROAT_H, "skimmer_throat (ctx)", &mut ok);
	let ring_seat_z = FLANGE_Z0 - RING_H as f32; // ring top = flange underside
	let throat_top_z = ring_seat_z; // throat top = ring bottom
	let p_ring = translated(&m_ring, ring_seat_z);
	let p_throat = translated(&m_throat_raw, throat_top_z - THROAT_H as f32);
	relation("flange seated on hanger ring", &m_basket, &p_ring, true, &mut ok);
	relation("hanger ring seated on skimmer throat", &p_ring, &p_throat, true, &mut ok);
	relation("basket clears throat bore (hangs free)", &m_basket, &p_throat, false, &mut ok);
	relation("basket mid-drop through ring bore", &m_basket, &translated(&m_ring, 34.0), false, &mut ok);

	// posed component STLs (in-use pose, one file per component) + merged scene
	let mut asm = Mesh::new();
	for (name, m) in [("basket", &m_basket), ("hanger_ring", &p_ring), ("skimmer_throat", &p_throat)] {
		let _ = std::fs::write(format!("pool_system/tpms_basket/assembly_parts/{name}.stl"), m.to_stl_binary());
		merge_into(&mut asm, m);
	}
	let _ = std::fs::write("pool_system/tpms_basket/ASSEMBLY.stl", asm.to_stl_binary());

	println!("\nTPMS BASKET: {}", if ok { "ALL GATES PASS" } else { "<<< FAIL" });
	std::process::exit(if ok { 0 } else { 1 });
}
