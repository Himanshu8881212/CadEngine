//! `StrutLattice` periodic strut lattices: every [`StrutKind`] must be exactly
//! 1-Lipschitz (secant-probed), hold its PINNED solid fraction, and mesh CLOSED
//! under a shroud; the tiling must be seam-free (an 8-cell block measures 8× a
//! 1-cell block and the field is continuous across every cell border); and
//! `pipe_path` must reproduce the exact right-angle capsule-chain volume.

use std::f64::consts::PI;

use kernel_implicit::strut::{graph_lattice, pipe_path, probe_lipschitz, StrutKind, StrutLattice};
use kernel_implicit::{check_mesh, manifold_dual_contour, Aabb, Cuboid, Node, Resolution, Sdf, Vec3};

#[test]
fn strut_kinds_one_lipschitz_mesh_closed_and_pinned_solid_fraction() {
	// Pinned solid fractions at cell = 10, radius = 1, measured on the
	// deterministic 30³ midpoint grid below (2026-07-29). Sanity arithmetic:
	// per period BCC has 8 interior struts of length 10·√3/2 (naive
	// Σπr²L/cell³ = 21.8% → measured 19.8% after junction overlap), FCC 12
	// full-cylinder equivalents of length 10/√2 (26.7% naive → 22.1%; its 24
	// struts lie IN the cell faces, each shared by two cells), Octet those 12
	// plus the 12 interior octahedron edges (53.3% naive → 39.3% — junctions
	// overlap heavily, consistent with lattice.rs's measured octet ratios).
	// Honest note: FCC and Octet sit ABOVE the ~3–20% band a slender lattice
	// suggests, because at radius/cell = 0.1 these cells are simply that dense
	// — the pin records what IS true.
	let pinned = [(StrutKind::Bcc, 0.198f32), (StrutKind::Fcc, 0.221), (StrutKind::Octet, 0.393)];

	let mut report = String::new();
	let mut all_ok = true;
	for (kind, want_frac) in pinned {
		let region = Aabb::new(Vec3::splat(-13.0), Vec3::splat(13.0));
		let lat = StrutLattice::new(region, kind, 10.0, 1.0);

		// (1) Exactly 1-Lipschitz: the secant probe may exceed 1 only by
		// floating-point rounding (0.5% slack), and must actually OBSERVE the
		// unit slope somewhere (≥ 0.95 floor — strut walls and node balls
		// expose unit-gradient regions in every probe direction family).
		let lip = probe_lipschitz(&lat, Aabb::new(Vec3::splat(-5.0), Vec3::splat(15.0)), 16);

		// (2) Solid fraction over one period, deterministic 30³ midpoint grid.
		let n = 30usize;
		let step = 10.0 / n as f32;
		let mut inside = 0u32;
		for i in 0..n {
			for j in 0..n {
				for k in 0..n {
					let p = Vec3::new(i as f32 + 0.5, j as f32 + 0.5, k as f32 + 0.5) * step;
					if lat.distance(p) < 0.0 {
						inside += 1;
					}
				}
			}
		}
		let frac = inside as f32 / (n * n * n) as f32;

		// (3) Bounded by a shroud cube, the lattice meshes CLOSED via the same
		// mesher the TPMS gate uses (min-union junctions are exactly why MDC is
		// the designated extractor).
		let solid = Node::primitive_bound(lat.clone()).intersection(Node::primitive(Cuboid::new(Vec3::ZERO, Vec3::splat(12.0))));
		let mesh = manifold_dual_contour(&solid, region, Resolution::VoxelSize(0.35));
		let rep = check_mesh(&mesh);

		let ok = (0.95..=1.005).contains(&lip)
			&& (frac - want_frac).abs() <= 0.01
			&& rep.boundary_edges == 0
			&& mesh.triangle_count() > 0;
		all_ok &= ok;
		report += &format!(
			"\n  {kind:?}: secant_lipschitz={lip:.4} (want 0.95..=1.005) solid_frac={frac:.3} (pinned {want_frac:.3} ±0.01) boundary_edges={} tris={} images={}",
			rep.boundary_edges,
			mesh.triangle_count(),
			lat.image_count()
		);
	}
	assert!(
		all_ok,
		"periodic strut lattices must be exactly 1-Lipschitz (secant probe), hold their pinned solid fractions, and mesh closed:{report}"
	);
}

#[test]
fn tiling_is_seam_free_eight_cells_measure_eight_times_one() {
	// The classic tiling bug is a strut not being seen from the neighboring
	// cell: the field jumps at the border, meshes crack at every seam, and the
	// 8-cell block loses (or gains, via spurious end caps) volume. Gates, per
	// lattice: (1) mesh a 1-cell block [0,10]³ and a 2×2×2-cell block [0,20]³
	// at identical settings — volumes must satisfy V8 ≈ 8·V1 within 2% (exact
	// mathematically: the solid volume inside ANY axis-aligned period cube is
	// fraction × cell³, so only voxelization error remains); (2) the big mesh
	// is CLOSED (no seam cracks); (3) the raw field is continuous across the
	// x/y/z = 10 borders (|Δd| over a ±5e-4 straddle ≤ the Lipschitz-allowed
	// 1e-3, plus f32 slack). The custom `graph_lattice` case is the sharpest:
	// one edge (0,½,½)→(1,½,½) must tile into CONTINUOUS infinite rods, so its
	// 1-cell volume must equal π·r²·cell — spurious per-cell end caps (the bug)
	// would add ~13%.
	let region = Aabb::new(Vec3::splat(-2.0), Vec3::splat(22.0));
	let rod_edges = [(Vec3::new(0.0, 0.5, 0.5), Vec3::new(1.0, 0.5, 0.5))];
	let cases: [(&str, StrutLattice, Option<f64>); 4] = [
		("Bcc", StrutLattice::new(region, StrutKind::Bcc, 10.0, 1.0), None),
		("Fcc", StrutLattice::new(region, StrutKind::Fcc, 10.0, 1.0), None),
		("Octet", StrutLattice::new(region, StrutKind::Octet, 10.0, 1.0), None),
		("x-rod graph_lattice", graph_lattice(&rod_edges, 10.0, 1.0), Some(PI * 10.0)),
	];

	let block = |lat: &StrutLattice, cells: f32| -> (f64, usize) {
		let half = 5.0 * cells;
		let solid = Node::primitive_bound(lat.clone()).intersection(Node::primitive(Cuboid::new(Vec3::splat(half), Vec3::splat(half))));
		let domain = Aabb::new(Vec3::splat(-1.37), Vec3::splat(2.0 * half + 1.37));
		let mesh = manifold_dual_contour(&solid, domain, Resolution::VoxelSize(0.31));
		(mesh.signed_volume(), check_mesh(&mesh).boundary_edges)
	};

	let mut report = String::new();
	let mut all_ok = true;
	for (name, lat, want_v1) in &cases {
		let (v1, _) = block(lat, 1.0);
		let (v8, bnd8) = block(lat, 2.0);
		let ratio = v8 / v1;

		let eps = 5e-4f32;
		let mut seam = 0.0f32;
		for axis in 0..3usize {
			for u in 0..24 {
				for v in 0..24 {
					let (cu, cv) = (0.4 + u as f32 * 0.8, 0.4 + v as f32 * 0.8);
					let mut lo = [0.0f32; 3];
					lo[axis] = 10.0 - eps;
					lo[(axis + 1) % 3] = cu;
					lo[(axis + 2) % 3] = cv;
					let mut hi = lo;
					hi[axis] = 10.0 + eps;
					seam = seam.max((lat.distance(Vec3::from_array(lo)) - lat.distance(Vec3::from_array(hi))).abs());
				}
			}
		}

		let v1_ok = match want_v1 {
			Some(want) => (v1 - want).abs() / want < 0.025,
			None => true,
		};
		let ok = (ratio - 8.0).abs() / 8.0 < 0.02 && bnd8 == 0 && seam <= 1.2e-3 && v1 > 0.0 && v1_ok;
		all_ok &= ok;
		report += &format!(
			"\n  {name}: V1={v1:.1} V8={v8:.1} ratio={ratio:.3} (want 8 ±2%) big_boundary_edges={bnd8} seam_jump={seam:.2e} (want ≤1.2e-3){}",
			match want_v1 {
				Some(want) => format!(" V1_analytic={want:.1} (want ±2.5%)"),
				None => String::new(),
			}
		);
	}
	assert!(
		all_ok,
		"tiling must be seam-free: 8-cell block ≈ 8× the 1-cell block, closed big mesh, field continuous across cell borders, rods continue through seams:{report}"
	);
}

#[test]
fn pipe_path_elbow_meshes_closed_at_exact_capsule_chain_volume() {
	// L-shaped 3-point path, r = 2: two perpendicular capsules sharing the
	// elbow point B. EXACT union volume (derivation): each capsule is
	// π·r²·L + (4/3)π·r³; their intersection, split by the elbow quadrants, is
	// three quarter-balls around B (π·r³) plus the quarter-Steinmetz where the
	// two cylinder bodies cross ((4/3)·r³ of the 16r³/3 bicylinder). So
	//   V = π·r²·(L1+L2) + (5/3)·π·r³ − (4/3)·r³  = 471.04 for L=20,15, r=2.
	// The 5% band is honest headroom for voxel quantization of the meshed
	// volume (measured deviation is well under 1% at voxel 0.25 — the assert
	// message prints it); the formula itself is exact for a 90° elbow.
	let pts = [Vec3::ZERO, Vec3::new(20.0, 0.0, 0.0), Vec3::new(20.0, 15.0, 0.0)];
	let pipe = pipe_path(&pts, 2.0);
	let lip = probe_lipschitz(&pipe, pipe.bounds().pad(2.0), 12);
	let mesh = manifold_dual_contour(&pipe, pipe.bounds().pad(1.13), Resolution::VoxelSize(0.25));
	let rep = check_mesh(&mesh);
	let vol = mesh.signed_volume();
	let r = 2.0f64;
	let want = PI * r * r * (20.0 + 15.0) + 5.0 / 3.0 * PI * r.powi(3) - 4.0 / 3.0 * r.powi(3);
	assert!(
		mesh.is_watertight()
			&& rep.boundary_edges == 0
			&& rep.non_manifold_edges == 0
			&& (0.95..=1.005).contains(&lip)
			&& (vol - want).abs() / want < 0.05,
		"pipe_path elbow: watertight={} bnd={} nme={} secant_lipschitz={lip:.4} (want 0.95..=1.005) vol={vol:.2} vs exact {want:.2} ({:+.2}%, want ±5%)",
		mesh.is_watertight(),
		rep.boundary_edges,
		rep.non_manifold_edges,
		(vol / want - 1.0) * 100.0
	);
}
