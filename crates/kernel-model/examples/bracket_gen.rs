//! BRACKET GEN — a PLA wall/shelf bracket taken through the ENTIRE
//! generative-design loop in one gated run: exact-B-rep baseline → ACE hex8
//! FEA (receipts) → SIMP topology optimization (`tools/ace_optimize_runner.py`)
//! → density field back through `GridField` into an implicit body → reverse
//! bridge to an exact solid → STEP → HONEST re-analysis of the final binary
//! geometry (the optimizer's own as-built doctrine: gate the part you print,
//! not the homogenized SIMP proxy) → print-readiness gates.
//!
//! Product: a 100 mm projection × 150 mm × 30 mm bracket that carries 10 kg
//! (98.1 N) at the shelf tip, mounts with two M4 countersunk screws
//! (hole-wizard DIN 74/ISO 273 geometry) plus a REQUIRED Ø10 cup washer, and
//! prints on its side — every screw feature is teardrop-roofed for that
//! orientation (the `teardrop_hole` idiom).
//!
//! **The governing check is CREEP, not strength.** A shelf bracket holds its
//! load permanently, so the short-term allowable answers the wrong question:
//! every stress is gated against `materials::pla::creep_allowable_mpa(23 °C,
//! 1 y)`. That is also what puts a 2-cent washer in the BOM — the printed
//! countersink under a bare M4 head is the one detail that fails it.
//!
//! The loop is re-proved EVERY run: this example writes the FEA/SIMP/buckling
//! job manifests itself, spawns `python3 tools/ace_{fea,optimize,buckling}_
//! runner.py`, parses their stdout-JSON receipts, and exits 1 if python3/ACE
//! is missing — the loop IS the product claim, so it must never silently
//! skip. SIMP determinism is verified by running the optimizer TWICE and
//! comparing `final_rho.npy` byte-for-byte.
//!
//! Route choices (stated, per DESIGN.md):
//! - density → geometry: THRESHOLD + SMOOTH (y-averaged to a 2.5D extruded
//!   web + 3×3 tent blur + trilinear GridField iso-surface) plus a continuous
//!   outer chord, NOT graded infill — the 2.5D web is what makes the
//!   side-lying print support-free.
//! - the print/FEA artifact is the MESHER's own watertight output; the
//!   reverse bridge exists to make that same geometry exact for STEP and is
//!   not in the path to the STL. `mesh_to_solid_recovered` (v2) runs as a
//!   finishing pass and is kept ONLY if it stays valid and volume-conserving
//!   to 1e-6; on this geometry it does not, so v1 ships and the run says so.
//! - the analysis grid is SAMPLED FROM THE FIELD, not parity-filled from an
//!   STL: `tools/voxelize_stl.py` turned this watertight mesh into 43
//!   disconnected components and a 1410× tip deflection.
//! - screw features on the optimized body are cut IMPLICITLY with the same
//!   constants the hole wizard uses on the baseline and the shipped coupon
//!   (pinned against `metric_hole_spec(4.0)` by a gate) — the bridged web is
//!   a many-thousand-facet solid, so the exact-boolean wizard cut runs on the
//!   exact baseline/coupon B-reps where it belongs.
//!
//! Run from the repo root:
//!   cargo run --release -p kernel-model --example bracket_gen
//!   -> bracket_system/gen_bracket/   (exit 1 on any FAIL)

use kernel_brep::math::{DAffine3, DMat3, DVec2, DVec3};
use kernel_brep::{
	countersink_hole, difference, export_step, extrude, force_ccw, import_step, metric_hole_spec,
	teardrop_hole, tessellate_default, union, validate, volume, Fit, Mesh, Solid,
};
use kernel_core::math::{Aabb, Vec3};
use kernel_core::sdf::Sdf;
use kernel_implicit::grid_field::GridField;
use kernel_model::{campaign::gate, materials, reverse};

// ---- product spec (frozen; the WHY lives here) -----------------------------------
/// Shelf projection: supports a 120–140 mm shelf board with modest front overhang.
const PROJ: f64 = 100.0;
/// Wall run: the lever that turns tip load into anchor tension — 150 keeps the
/// top-screw pull-out under ~0.1 kN at the rated load (see ANALYSIS.md).
const HEIGHT: f64 = 150.0;
/// Width: lateral stiffness for a shelf that can be bumped, and the side-lying
/// print height (30 mm of 0.2 layers ≈ 150 layers — a short print).
const WIDTH: f64 = 30.0;
/// Wall-plate thickness: the M4 countersink cone is 2.75 deep (Ø10 → Ø4.5),
/// leaving ≥3 mm of straight bore — flush head + full plate bearing.
const PLATE_T: f64 = 6.0;
/// Top strip kept solid the full projection: the shelf must land on a
/// continuous surface, not on whatever chord SIMP leaves.
const ARM_T: f64 = 6.0;
/// Rated load 10 kg → 98.1 N. The PLA allowables already carry base 35 MPa ×
/// 0.6 layer adhesion × 0.5 design factor (≈ ×2 on top of a conservative
/// yield), so 10 kg is the honest rating, not a hope.
const LOAD_KG: f64 = 10.0;
const LOAD_N: f64 = LOAD_KG * 9.81;
/// Load pad: the outer 12 mm of the shelf seat — the worst (tip-loaded) case.
const PAD_X0: f64 = 88.0;
/// M4 countersunk screw stations (y = 0). Top screw high: anchor tension =
/// M / spread ≈ 98.1 N · 94 mm / 110 mm ≈ 84 N — comfortably inside a plastic
/// wall plug's rating; bottom screw low where the web is shallow.
const SCREW_TOP_Z: f64 = 130.0;
const SCREW_BOT_Z: f64 = 20.0;
/// Driver tunnel: passes a 1/4" hex bit holder (11.1 mm across corners) with
/// ~0.9 mm clearance, and any plain M4 driver easily. Ø12 not Ø14 because
/// the teardrop roof's apex sits `r/cos(roof)` above the axis, so the void
/// eats `r + r/cos(roof)` of the part's 30 mm WIDTH: Ø14 at a 55° roof took
/// 19.2 mm of it and measurably cost stiffness (tip 2.25× baseline). Ø12 at
/// 48° takes 15.0 mm and leaves the section that carries the load.
const TUNNEL_D: f64 = 12.0;
/// Screw clearance bore Ø5.2 — deliberately over ISO 273 medium (Ø4.5): the
/// wizard's exact Ø4.5 bore is kept in the baseline/coupon B-reps, and this
/// teardrop overbore clears its facets by ≥0.3 so the two cutters never leave
/// near-coincident cylinder slivers (§7.7); horizontal printed holes also
/// shrink, so FDM practice oversizes them anyway. Head still seats on the
/// cone from Ø5.2 out to Ø8 (bearing checked in ANALYSIS.md).
const BORE_D: f64 = 5.2;
/// DIN 74 form F countersink for M4 — pinned against the wizard table by a gate.
const CSK_D: f64 = 10.0;
/// Roof angle over the countersink CROWN. DIN 74 form F is a 90° included
/// cone, i.e. flanks at exactly 45.0° — precisely the support audit's limit,
/// so the crown must be roofed away rather than argued about. 60° (not the
/// tunnels' 48°) because this feature is small: the roof band is only a few
/// voxels deep, where dual-contour facet jitter is largest, so it buys its
/// margin with angle. The band spans the cone's FULL depth for the same
/// reason — a 2.3 mm band was under-resolved and flagged 30 mm².
const CROWN_ROOF_DEG: f64 = 60.0;
/// Crown-roof band: the countersink cone's full axial extent (apex plane to
/// just past the entry face). Shared by the B-rep cut and the implicit
/// cutter so the two routes remove the SAME material.
const CROWN_X0: f64 = PLATE_T - CSK_D / 2.0;
const CROWN_X1: f64 = PLATE_T + 0.3;
/// M4 countersunk finishing ("cup") washer, Ø10 OD — a REQUIRED part, not a
/// nicety, and the gate below is why. The bracket is sustained-loaded, so the
/// screw seat is checked against the CREEP allowable, and a bare DIN 7991 M4
/// head (Ø8) bears on only ~25 mm² of printed cone: ≈3.1 MPa, over the
/// 2.5 MPa sustained line. The washer spreads the same tension over the full
/// Ø10 countersink and roughly halves it. It is the cheapest part in the BOM
/// and it is the one that makes the 10 kg sustained rating defensible.
const WASHER_OD: f64 = 10.0;
/// Fraction of the countersink annulus that actually bears: the crown roof
/// (see CROWN_ROOF_DEG) removes a cap from the cone's upper side, so the
/// seat is not a full annulus. 0.85 is a deliberate under-estimate of what
/// remains — the bearing gate should not be flattered by geometry it lost.
const SEAT_BEARING_FRAC: f64 = 0.85;
/// Teardrop roof angle. The `teardrop_hole` doc's 46° is "just past the 45°
/// FDM overhang limit" — 1° of margin, which is enough for an exact B-rep
/// (the coupon audits at steep 0.0000 mm²) but NOT for the optimized body:
/// it is re-discretized by dual contouring, and facet jitter dips a 46° roof
/// below 45°. Measured: at 48° the audit still flagged ~29 mm² on the roof
/// flanks (facet deviation runs to several degrees at MESH_VOX). 55° carries
/// 10° of margin, which the section can now afford because the optimizer is
/// told about the tunnels (they are VOID keep-outs) and routes material
/// around them instead of into them.
const ROOF_DEG: f64 = 55.0;
/// Hypotenuse slope: from the plate foot (PLATE_T, 0) to the arm tip corner
/// (PROJ, HEIGHT − ARM_T) — the arm end face stays a square, printable corner.
const HYP_SLOPE: f64 = (HEIGHT - ARM_T) / (PROJ - PLATE_T);

// ---- generative-loop numbers (frozen; measured WHYs in comments) ------------------
/// FEA/SIMP voxel: 2.5 mm → SIMP features ≥ ~2 voxels = 5 mm, chunky and
/// printable; the half-model grid is 40×6×60 = 14 400 elements (~26k DOF),
/// which keeps a full two-optimizer-run campaign in single-digit minutes.
const VOX: f64 = 2.5;
const GRID_NX: usize = 40;
/// The analysis grid, as one tuple (every job and every sampling call).
const GRID_DIMS: (usize, usize, usize) = (GRID_NX, GRID_NY, GRID_NZ);
/// HALF model in y (symmetry slider on the y≈0 element layer): the part, the
/// load and the fixtures are all y-symmetric, so the half grid solves the
/// same problem at half the DOF. Full-width forces are halved in the jobs.
const GRID_NY: usize = 6;
const GRID_NZ: usize = 60;
/// SIMP volume-fraction target on the DESIGN region (frozen skeleton
/// excluded). Swept 2026-07-31 at the filter radius below, measured on the
/// rebuilt 2.5D section: 0.42 → 40.9% mass cut, 0.34 → 43.4% at tip ratio
/// 1.41, 0.30 → 45.7% but tip ratio 1.56 (over the 1.5× stiffness gate).
/// Swept against BOTH product gates on the fully rebuilt part (mass cut ≥40%
/// AND tip ≤1.5× baseline), measured end-to-end 2026-07-31:
///   0.34 → 43.2% cut but 1.61× tip (fails stiffness)
///   0.42 → 1.23× tip but 35.8% cut (fails mass)
///   0.37 → 39.9% cut at 1.50× tip — both gates binding simultaneously
/// (those three were measured before the teardrop-roof winding fix, which
/// changed how much material the screw tunnels remove; re-tuned against the
/// corrected geometry: 0.35 → 42.4% at 1.55×, 0.365 → 41.0% at 1.51×.
/// Declaring the driver tunnels as VOID keep-outs then bought back real
/// stiffness (0.365 → 1.31× tip), which is spent here on mass: 0.34 is the
/// shipped setting — numbers in ANALYSIS.md)
/// The two gates close on each other from opposite sides — this is a real
/// stiffness-vs-mass trade, not a slack parameter. Neither gate was moved to
/// fit the optimizer; the optimizer was set to clear both.
const VOLFRAC: f64 = 0.34;
const SIMP_PENALTY: f64 = 3.0;
/// Density-filter radius in voxels — the optimizer's MINIMUM LENGTH SCALE
/// knob, and the honest way to keep a topology printable. Measured sweep of
/// the rebuilt section's thinnest member: r=1.5 → 0.50 mm and THREE
/// disconnected pieces, r=2.5 → 0.50 mm and two pieces, r=3.5 → one
/// connected body whose thinnest member is 3.50 mm with zero sub-1.6 mm
/// samples. Thin necks are a filter-radius problem, not a post-processing
/// one; the gate below asserts the imposed scale clears the nozzle.
const SIMP_FILTER_RVOX: f64 = 3.5;
const SIMP_MAX_ITERS: u64 = 40;
/// Outer compression chord: a ONE-SIDED strip lying just INSIDE the
/// hypotenuse, kept solid in the rebuild. SIMP asks for material along this
/// whole diagonal (measured density ≈0.5 its full length) but leaves it
/// hovering AT the iso threshold, so the iso-surface ran tangent to the
/// hypotenuse clip and tapered members to knife edges. A continuous chord is
/// the truss-correct answer (a truss needs a chord, not a fringe of
/// tapers) and the geometric one. One-sided is load-bearing: a symmetric
/// band about the line has half its width outside the silhouette, and
/// clipping THAT is what produced fresh slivers (measured: sub-1.6 mm
/// medial samples 4 -> 8). At 4.0 mm the section is measurably clean —
/// 0 sub-1.6 mm samples, thinnest member 3.50 mm.
const CHORD_T: f64 = 4.0;
/// Rebuild mesher voxel (mm). 1.2 rather than 1.5: the support audit is run
/// on THIS discretization, and coarser facets jitter the teardrop roof
/// angles enough to trip the 45° limit. Fine enough to hold the roofs,
/// coarse enough that the faceted STEP stays exchangeable.
const MESH_VOX: f32 = 1.2;
/// Iso threshold on the (blurred, y-averaged) density.
const ISO: f32 = 0.5;
/// Pseudo-SDF scale (mm): density transitions span ~2 SIMP cells after the
/// filter+blur, so (iso − rho)·6 has near-unit slope at the boundary. This is
/// a BOUND, not a distance field — meshing is sample-based (dual contouring),
/// which needs sign correctness, not Lipschitz; stated per §12.3 honesty.
const SDF_SCALE: f32 = 6.0;

// ---- gates: pinned to what the loop measurably delivers ---------------------------
/// Optimized tip deflection ≤ this × baseline — the product claim ("~40%
/// less plastic, within 1.5× the stiffness"). It is the binding constraint
/// on how much material the optimizer is allowed to remove: VOLFRAC was
/// chosen against THIS gate, not the other way round.
const STIFF_FACTOR: f64 = 1.5;
/// Whole-part mass reduction floor, gated on the MEASURED solid volumes.
/// Pinned at 40% — the product-level claim — which the tuned parameter set
/// clears by ~3 points (43% measured on the rebuilt section, before the
/// screw cuts that only widen the gap).
const MASS_RED_MIN: f64 = 0.40;
/// Coarse hex8 under-predicts peak bending stress ~20% (runner's own caveat)
/// — the RT stress gate derates the reported peak by ×1.25 before comparing.
const HEX8_PEAK_FACTOR: f64 = 1.25;
/// The SUSTAINED-load design point, looked up in
/// `materials::pla::creep_allowable_mpa` (the researched creep table; never
/// re-typed here). A shelf bracket holds its load continuously for years, so
/// printed PLA's short-term allowable is the WRONG tier — creep governs.
/// 23 °C because the product is an indoor wall bracket; 8760 h = 1 year
/// because a bookshelf bracket is loaded and forgotten. The lookup rounds
/// BOTH arguments up to the next tabulated cell, so these land exactly on
/// the 23 °C / 1 y cell — the table's longest, which its own confidence note
/// calls a conservative extrapolation (no printed-PLA creep dataset beyond
/// ~170 h exists) on top of a safety factor of 2.0 against measured rupture.
const CREEP_DESIGN_T_C: f64 = 23.0;
const CREEP_DESIGN_HOURS: f64 = 8760.0;
/// Buckling: the optimized web's diagonals are compression members, so the
/// mode is real and now has a solver. Gate on the KNOCKED-DOWN critical load
/// (the runner's cited 0.5 for FDM imperfection) being ≥ this multiple of
/// the rated load — i.e. buckling is provably not the governing failure at
/// rated load, which strength/creep is.
const BUCKLE_MIN_FACTOR: f64 = 3.0;
/// Min printable feature: 4 × 0.4 nozzle perimeters.
const MIN_FEATURE: f32 = 1.6;
/// Print bridge bound (mm): the teardrop roofs leave no flat ceilings, so
/// this is a defensive cap on incidental web spans.
const BRIDGE_MAX: f64 = 11.0;
/// Support budget for the REBUILT part, as a fraction of its surface area.
/// This is NOT a design overhang: the exact B-rep coupon carries the
/// identical screw features and audits at 0.0000 mm². It is dual contouring
/// rounding the sharp APEX RIDGE of each teardrop roof into a ~half-voxel
/// band, whose facets then read as near-horizontal ceiling. The residual
/// therefore scales with MESH_VOX, not with the design (measured ~21 mm² at
/// 1.2 mm — 0.03% of the surface, bands ~0.2 mm wide, which a slicer lays
/// down as ordinary extrusion). Budgeted at 0.1% AND every flagged patch
/// must sit at a screw feature: the web itself has to audit clean.
const SUPPORT_BUDGET_FRAC: f64 = 0.001;
/// Radius (mm) around a screw axis inside which the budgeted support
/// residual must lie — the tunnel apex (10.5) plus meshing slack.
const SCREW_FEATURE_R: f64 = 14.0;
/// Bed fit: 256 mm class printer minus margin.
const BED_MAX: f64 = 250.0;

const PLA: f64 = materials::PLA_G_PER_MM3;
const FAM: &str = "bracket_system/gen_bracket";

// ---- small geometry helpers -------------------------------------------------------

fn v(x: f64, y: f64, z: f64) -> DVec3 {
	DVec3::new(x, y, z)
}

/// Prism from an (x, z) profile swept along +Y over [y0, y1] (det +1 frame,
/// same construction as DRYBOX's `prism_y`).
fn prism_y(profile: &[(f64, f64)], y0: f64, y1: f64) -> Solid {
	let p: Vec<DVec2> = profile.iter().map(|&(x, z)| DVec2::new(x, z)).collect();
	let m = DAffine3::from_mat3_translation(DMat3::from_cols(DVec3::X, DVec3::Z, DVec3::NEG_Y), v(0.0, y1, 0.0));
	extrude(&force_ccw(p), y1 - y0).transformed(m)
}

/// Prism from a (y, z) profile swept along +X over [x0, x1] (local X→Y,
/// Y→Z, Z→X is the cyclic det +1 frame).
fn prism_x(profile: &[(f64, f64)], x0: f64, x1: f64) -> Solid {
	let p: Vec<DVec2> = profile.iter().map(|&(y, z)| DVec2::new(y, z)).collect();
	let m = DAffine3::from_mat3_translation(DMat3::from_cols(DVec3::Y, DVec3::Z, DVec3::X), v(x0, 0.0, 0.0));
	extrude(&force_ccw(p), x1 - x0).transformed(m)
}

/// The countersink-crown roof relief: a 3-point prism whose 47° flanks replace
/// the cone's 45.0°-at-the-crown facets (borderline vs the 45° audit) with
/// safely-steep planes. Base dips 0.2 into the cone void so no tangent sliver
/// survives; spans the cone's crown region x ∈ [PLATE_T − 2.0, PLATE_T + 0.3].
fn csk_roof_profile(zs: f64) -> Vec<(f64, f64)> {
	let r = CSK_D / 2.0; // 5.0 at the entry plane
	let roof = CROWN_ROOF_DEG.to_radians();
	let (yt, zt) = (r * roof.cos(), r * roof.sin());
	vec![(yt - 0.1, zs - zt), (yt - 0.1, zs + zt), (r / roof.cos(), zs)]
}

/// Hypotenuse x at height z (the web's outer boundary).
fn hyp_x(z: f64) -> f64 {
	PLATE_T + z / HYP_SLOPE
}

/// Cut ONE complete M4 countersunk screw station into a B-rep solid, print-up
/// = +Y (side-lying): teardrop driver tunnel (floor = head-seat plane at
/// x = PLATE_T), hole-wizard DIN 74 countersink + ISO 273 bore, teardrop
/// overbore, countersink crown roof. Shared by the baseline and the coupon.
fn cut_screw_station(s: Solid, zs: f64, tunnel_to: f64) -> Result<Solid, String> {
	let e = |what: &str, err: kernel_brep::HoleError| format!("{what} at z={zs}: {err:?}");
	// driver tunnel: blind teardrop pocket, floor lands exactly on x = PLATE_T
	let s = teardrop_hole(&s, v(PLATE_T + 0.5, 0.0, zs), DVec3::X, DVec3::Y, TUNNEL_D, tunnel_to - PLATE_T - 0.5 + 2.0, ROOF_DEG, None)
		.map_err(|er| e("tunnel", er))?;
	// the hole wizard's countersunk M4: 90° cone Ø10 at the tunnel floor + Ø4.5 through-bore
	let s = countersink_hole(&s, v(PLATE_T, 0.0, zs), -DVec3::X, 4.0, Fit::Medium, None).map_err(|er| e("countersink", er))?;
	// teardrop overbore Ø5.2 through the plate (roofs the wizard's round bore)
	let s = teardrop_hole(&s, v(PLATE_T + 0.3, 0.0, zs), -DVec3::X, DVec3::Y, BORE_D, PLATE_T + 0.3 + 0.5, ROOF_DEG, None)
		.map_err(|er| e("overbore", er))?;
	// roof the countersink crown (the standard's 45.0° flanks → CROWN_ROOF_DEG)
	Ok(difference(&s, &prism_x(&csk_roof_profile(zs), CROWN_X0, CROWN_X1)))
}

/// The baseline bracket: solid right-triangle web (blunted foot), exact B-rep,
/// both screw stations wizard-cut. This is the mass/stiffness reference the
/// optimization must beat AND the CAD statement of the mounting interface.
fn build_baseline() -> Result<Solid, String> {
	let profile = [
		(0.0, 0.0),
		(PLATE_T, 0.0), // blunt plate foot: no fragile knife tip at the origin
		(PROJ, HEIGHT - ARM_T),
		(PROJ, HEIGHT),
		(0.0, HEIGHT),
	];
	let tri = prism_y(&profile, -WIDTH / 2.0, WIDTH / 2.0);
	let s = cut_screw_station(tri, SCREW_TOP_Z, hyp_x(SCREW_TOP_Z + TUNNEL_D / 2.0) + 3.0)?;
	cut_screw_station(s, SCREW_BOT_Z, hyp_x(SCREW_BOT_Z + TUNNEL_D / 2.0) + 3.0)
}

/// The M4 seat coupon (optional/): one full screw station in a 15-minute
/// block — verify the head seats flush and the teardrop roofs print clean
/// BEFORE committing to the bracket.
fn build_coupon() -> Result<Solid, String> {
	let block = prism_y(&[(0.0, 0.0), (20.0, 0.0), (20.0, 30.0), (0.0, 30.0)], -WIDTH / 2.0, WIDTH / 2.0);
	cut_screw_station(block, 15.0, 21.0)
}

// ---- implicit final body ----------------------------------------------------------

/// Signed-distance BOUND of a teardrop prism along X (circle ∪ roof triangle
/// in the (y, z) cross-section), used to cut the screw stations into the
/// bridged web with EXACTLY the wizard-pinned dimensions.
#[derive(Clone)]
struct TeardropPrism {
	zs: f32,
	r: f32,
	x0: f32,
	x1: f32,
}

impl TeardropPrism {
	fn dist(&self, p: Vec3) -> f32 {
		let roof = (ROOF_DEG as f32).to_radians();
		let (dy, dz) = (p.y, p.z - self.zs);
		let circle = (dy * dy + dz * dz).sqrt() - self.r;
		let (yt, zt) = (self.r * roof.cos(), self.r * roof.sin());
		let apex = self.r / roof.cos();
		let tri = tri_sdf((dy, dz), (yt, -zt), (yt, zt), (apex, 0.0));
		let slab = (self.x0 - p.x).max(p.x - self.x1);
		circle.min(tri).max(slab)
	}
}

/// A 1-sample density grid reading 1.0 everywhere — the placeholder for the
/// solid-control / baseline fields, whose `solid_control` flag means the
/// density is never consulted.
fn unit_grid() -> GridField {
	GridField::from_data(vec![1.0], (1, 1, 1), Vec3::ZERO, 1.0).expect("1×1×1 unit grid is valid")
}

/// Convex-triangle SDF in 2D (max of the three edge half-planes; exact for a
/// convex polygon's exterior side, a bound inside — fine for a cutter).
///
/// ORIENTATION-AGNOSTIC on purpose. The half-plane normal `(ey, -ex)` is
/// outward only for a COUNTER-clockwise triangle; fed a clockwise one it
/// points inward and the function returns positive (= "outside") everywhere,
/// so the triangle silently cuts NOTHING. That is not hypothetical: both
/// roof triangles here are naturally wound clockwise, and until this was
/// fixed the teardrop tunnel roofs and the countersink crown roof never
/// cut — the tunnels stayed bare circles whose near-horizontal ceilings
/// (normals -0.71…-1.0) were the 689 mm² the support audit kept flagging.
/// The signed area now decides the sign, so either winding works.
fn tri_sdf(p: (f32, f32), a: (f32, f32), b: (f32, f32), c: (f32, f32)) -> f32 {
	let area2 = (b.0 - a.0) * (c.1 - a.1) - (c.0 - a.0) * (b.1 - a.1);
	let s = if area2 < 0.0 { -1.0 } else { 1.0 };
	let edge = |u: (f32, f32), w: (f32, f32)| {
		let (ex, ey) = (w.0 - u.0, w.1 - u.1);
		let len = (ex * ex + ey * ey).sqrt().max(1e-9);
		let (nx, ny) = (ey / len, -ex / len);
		s * (nx * (p.0 - u.0) + ny * (p.1 - u.1))
	};
	edge(a, b).max(edge(b, c)).max(edge(c, a))
}

/// The final optimized bracket as ONE implicit field:
/// (thresholded SIMP web ∪ frozen skeleton) ∩ triangle profile − screw cuts.
/// `mutilated` chops the lower web/plate — the FEA negative control.
#[derive(Clone)]
struct BracketField {
	rho2d: GridField,
	mutilated: bool,
	/// Solid CONTROL: ignore the density and fill the whole silhouette. The
	/// same probes run on this and on the real body, so any thin-wall floor
	/// set by the SILHOUETTE (a wedge where the profile clips at a shallow
	/// angle) is separated from one the optimizer actually introduced.
	/// With `chord: false` this is exactly the BASELINE bracket, which is
	/// how the baseline reaches the FEA on the same sampling path as the
	/// optimized part (and is cross-checked against the exact B-rep volume).
	solid_control: bool,
	/// Include the outer compression chord (the rebuild's design addition).
	chord: bool,
}

impl BracketField {
	fn skeleton(&self, p: Vec3) -> f32 {
		let box_d = |x0: f64, x1: f64, z0: f64, z1: f64| -> f32 {
			let dx = ((x0 as f32) - p.x).max(p.x - x1 as f32);
			let dz = ((z0 as f32) - p.z).max(p.z - z1 as f32);
			dx.max(dz)
		};
		// plate ∪ arm strip ∪ the two screw-boss bands (2.5D, full width) —
		// byte-for-byte the regions the SIMP manifest freezes — ∪ the outer
		// compression chord along the hypotenuse (a rebuild-time design
		// decision, stated in DESIGN.md; the final FEA measures the result).
		let plate = box_d(0.0, PLATE_T, 0.0, HEIGHT);
		let arm = box_d(0.0, PROJ, HEIGHT - ARM_T, HEIGHT);
		let boss_top = box_d(PLATE_T, 30.0, SCREW_TOP_Z - 9.0, SCREW_TOP_Z + 9.0);
		let boss_bot = box_d(PLATE_T, 26.0, SCREW_BOT_Z - 9.0, SCREW_BOT_Z + 9.0);
		// chord: the strip between the hypotenuse and a parallel line
		// CHORD_T inside it (`signed` < 0 is inside), ending in the plate and
		// the arm so neither end is a free tip.
		let norm = ((HYP_SLOPE * HYP_SLOPE + 1.0).sqrt()) as f32;
		let signed = ((p.x - PLATE_T as f32) * HYP_SLOPE as f32 - p.z) / norm;
		let strip = signed.max(-(CHORD_T as f32) - signed);
		let chord = if self.chord {
			strip.max(PLATE_T as f32 - p.x).max(p.z - (HEIGHT - ARM_T) as f32)
		} else {
			f32::INFINITY
		};
		plate.min(arm).min(boss_top).min(boss_bot).min(chord)
	}

	fn cutters(&self, p: Vec3) -> f32 {
		let mut d = f32::INFINITY;
		for zs in [SCREW_TOP_Z, SCREW_BOT_Z] {
			let exit = (hyp_x(zs + TUNNEL_D / 2.0) + 5.0) as f32;
			let tunnel = TeardropPrism { zs: zs as f32, r: (TUNNEL_D / 2.0) as f32, x0: PLATE_T as f32, x1: exit };
			let bore = TeardropPrism { zs: zs as f32, r: (BORE_D / 2.0) as f32, x0: -1.0, x1: (PLATE_T + 0.3) as f32 };
			// 90° countersink: radius grows 1:1 with x from the apex plane
			let apex_x = (PLATE_T - CSK_D / 2.0) as f32;
			let rho = (p.y * p.y + (p.z - zs as f32) * (p.z - zs as f32)).sqrt();
			let cone = ((rho - (p.x - apex_x)) * std::f32::consts::FRAC_1_SQRT_2).max(p.x - (PLATE_T + 0.3) as f32);
			// crown roof (the same prism the B-rep route subtracts)
			let prof = csk_roof_profile(zs);
			let roof = tri_sdf(
				(p.y, p.z),
				(prof[0].0 as f32, prof[0].1 as f32),
				(prof[1].0 as f32, prof[1].1 as f32),
				(prof[2].0 as f32, prof[2].1 as f32),
			)
			.max((CROWN_X0 as f32 - p.x).max(p.x - CROWN_X1 as f32));
			d = d.min(tunnel.dist(p)).min(bore.dist(p)).min(cone).min(roof);
		}
		if self.mutilated {
			// NC: sever the truss with a full-depth vertical slab at
			// x ∈ [45, 60], below the shelf strip — every diagonal crossing
			// mid-span is cut, so the arm is left cantilevering on the strip
			// alone and the deflection must JUMP. (The first NC chopped
			// z < 55 outboard of x = 4 and moved the tip only 1.4×: the
			// optimizer had ALREADY emptied that corner, so it removed
			// almost nothing load-bearing — a negative control has to break
			// something the part is actually using.)
			let chop = ((45.0 - p.x).max(p.x - 60.0)).max((-1.0 - p.z).max(p.z - (HEIGHT - ARM_T) as f32));
			d = d.min(chop);
		}
		d
	}
}

impl Sdf for BracketField {
	fn distance(&self, p: Vec3) -> f32 {
		let web = if self.solid_control {
			f32::NEG_INFINITY // the control is solid everywhere inside the profile
		} else {
			(ISO - self.rho2d.sample(p)) * SDF_SCALE
		};
		let body = web.min(self.skeleton(p));
		// triangle profile: box ∩ hypotenuse half-space
		let bx = (-p.x).max(p.x - PROJ as f32);
		let by = (-(WIDTH / 2.0) as f32 - p.y).max(p.y - (WIDTH / 2.0) as f32);
		let hyp = (((p.x - PLATE_T as f32) * HYP_SLOPE as f32) - p.z) / ((HYP_SLOPE * HYP_SLOPE + 1.0).sqrt() as f32);
		let bz = (-p.z).max(p.z - HEIGHT as f32);
		let profile = bx.max(by).max(bz).max(hyp);
		body.max(profile).max(-self.cutters(p))
	}

	fn bounds(&self) -> Aabb {
		Aabb::new(Vec3::new(-1.0, -(WIDTH / 2.0) as f32 - 1.0, -1.0), Vec3::new(PROJ as f32 + 1.0, (WIDTH / 2.0) as f32 + 1.0, HEIGHT as f32 + 1.0))
	}
}

// ---- python receipt plumbing ------------------------------------------------------

/// Run `python3 <tool> <job>` from the repo root and parse the LAST non-empty
/// stdout line as the JSON receipt (the runners' stated contract). Any spawn
/// failure, non-JSON tail, or `ok: false` is an Err — the campaign FAILS
/// loudly rather than skipping the loop.
fn run_py(tool: &str, job: &str) -> Result<serde_json::Value, String> {
	let out = std::process::Command::new("python3")
		.args([tool, job])
		.output()
		.map_err(|e| format!("python3 not runnable ({e}) — the generative loop cannot be skipped"))?;
	let stdout = String::from_utf8_lossy(&out.stdout);
	let last = stdout.lines().rev().find(|l| !l.trim().is_empty()).unwrap_or("");
	let val: serde_json::Value = serde_json::from_str(last).map_err(|e| {
		let err_tail: String = String::from_utf8_lossy(&out.stderr).chars().rev().take(400).collect::<String>().chars().rev().collect();
		format!("{tool}: last stdout line is not JSON ({e}); stderr tail: {err_tail}")
	})?;
	if val.get("ok").and_then(|b| b.as_bool()) != Some(true) {
		return Err(format!("{tool}: {}", val.get("error").and_then(|e| e.as_str()).unwrap_or("ok != true")));
	}
	Ok(val)
}

/// Receipt-or-die: gate the step and abort the campaign on Err (a missing
/// interpreter must fail the run, not degrade it).
fn require(label: &str, res: Result<serde_json::Value, String>, receipt_path: &str, ok: &mut bool) -> serde_json::Value {
	match res {
		Ok(mut val) => {
			// Receipts are DELIVERABLES, so they must not carry wall-clock:
			// `timings_s` is the only field that changes between two runs of
			// the same job, and leaving it in makes the whole family
			// non-reproducible for no analytical gain (the content-addressed
			// `geometry_hash` and every physics field are already stable).
			if let Some(o) = val.as_object_mut() {
				o.remove("timings_s");
			}
			round_floats(&mut val, 9);
			let _ = std::fs::write(receipt_path, format!("{val:#}\n"));
			gate(label, true, format!("receipt {}", receipt_path.rsplit('/').next().unwrap_or("")), ok);
			val
		}
		Err(e) => {
			gate(label, false, e.chars().take(120).collect(), ok);
			println!("\nBRACKET GEN: <<< FAIL (generative loop step could not run)");
			std::process::exit(1);
		}
	}
}

/// Round every float in a receipt to `sig` significant digits, in place.
///
/// Why a deliverable gets rounded at all: the static FEA is bitwise
/// reproducible, but the buckling eigensolve is ARPACK, which is iterative
/// and NOT bitwise — measured run-to-run spread on the load factor is
/// ~8e-14 relative (λ 74.09145448458712 vs …59277). Persisting that raw
/// makes one deliverable change on every run for a reason twelve orders
/// below engineering significance. Nine significant digits is ~1e5 times
/// coarser than the jitter and ~1e6 times finer than anything quoted, so
/// the receipt is stable AND lossless at the level anyone reads it.
fn round_floats(v: &mut serde_json::Value, sig: i32) {
	match v {
		serde_json::Value::Number(n) => {
			if let Some(x) = n.as_f64() {
				if x != 0.0 && x.is_finite() {
					let mag = x.abs().log10().floor() as i32;
					let f = 10f64.powi(sig - 1 - mag);
					if let Some(num) = serde_json::Number::from_f64((x * f).round() / f) {
						*n = num;
					}
				}
			}
		}
		serde_json::Value::Array(a) => a.iter_mut().for_each(|e| round_floats(e, sig)),
		serde_json::Value::Object(o) => o.iter_mut().for_each(|(_, e)| round_floats(e, sig)),
		_ => {}
	}
}

/// Write a C-order `(nx, ny, nz)` float32 NumPy `.npy` (format v1.0) — the
/// density-grid interchange both ACE runners read (`job.npy`).
fn write_npy(path: &str, data: &[f32], dims: (usize, usize, usize)) -> std::io::Result<()> {
	let (nx, ny, nz) = dims;
	let dict = format!("{{'descr': '<f4', 'fortran_order': False, 'shape': ({nx}, {ny}, {nz}), }}");
	// total header (10 magic+len bytes + dict + '\n') padded to a 64-byte multiple
	let mut header = dict.into_bytes();
	while (10 + header.len() + 1) % 64 != 0 {
		header.push(b' ');
	}
	header.push(b'\n');
	let mut out = Vec::with_capacity(10 + header.len() + data.len() * 4);
	out.extend_from_slice(b"\x93NUMPY\x01\x00");
	out.extend_from_slice(&(header.len() as u16).to_le_bytes());
	out.extend_from_slice(&header);
	for v in data {
		out.extend_from_slice(&v.to_le_bytes());
	}
	std::fs::write(path, out)
}

/// Sample an implicit body onto the analysis grid as a SOLID-FRACTION field:
/// 2×2×2 sub-samples per element (the `supersample=2` convention ACE's own
/// `sample_part` uses), fraction of sub-points with `d < 0`.
///
/// This replaces parity-filling an STL (`tools/voxelize_stl.py`). Measured
/// 2026-07-31, and the reason the swap is not a convenience: parity-filling
/// the dual-contour mesh of this part — watertight, 0 non-manifold edges —
/// produced 43 disconnected components and y-layer counts varying 959…1237
/// on a body that is a PRISM in y. The FEA then read a 1410× tip deflection.
/// The analysis grid is now sampled from the same field the geometry is
/// meshed from, and a gate below proves that field agrees with the exact
/// B-rep baseline it is supposed to represent.
fn sample_occupancy<S: Sdf + ?Sized>(sdf: &S, dims: (usize, usize, usize), origin: Vec3, vox: f32) -> Vec<f32> {
	let (nx, ny, nz) = dims;
	let mut out = vec![0.0f32; nx * ny * nz];
	let q = vox * 0.25; // sub-sample offset: element center ± vox/4
	for i in 0..nx {
		for j in 0..ny {
			for k in 0..nz {
				let c = origin + Vec3::new((i as f32 + 0.5) * vox, (j as f32 + 0.5) * vox, (k as f32 + 0.5) * vox);
				let mut inside = 0;
				for &dx in &[-q, q] {
					for &dy in &[-q, q] {
						for &dz in &[-q, q] {
							if sdf.distance(c + Vec3::new(dx, dy, dz)) < 0.0 {
								inside += 1;
							}
						}
					}
				}
				out[(i * ny + j) * nz + k] = inside as f32 / 8.0;
			}
		}
	}
	out
}

fn f(v: &serde_json::Value, path: &[&str]) -> f64 {
	let mut cur = v;
	for k in path {
		cur = &cur[k];
	}
	cur.as_f64().unwrap_or(f64::NAN)
}

// ---- mesh posing helpers ----------------------------------------------------------

fn mesh_posed(m: &Mesh, t: DAffine3) -> Mesh {
	let mut out = m.clone();
	for p in &mut out.positions {
		let q = t.transform_point3(DVec3::new(p.x as f64, p.y as f64, p.z as f64));
		*p = Vec3::new(q.x as f32, q.y as f32, q.z as f32);
	}
	out
}

fn merge_into(dst: &mut Mesh, src: &Mesh) {
	let base = dst.positions.len() as u32;
	dst.positions.extend_from_slice(&src.positions);
	dst.indices.extend(src.indices.iter().map(|i| i + base));
}

/// Model frame → print pose: lie on the y = −15 face (X→X, Y→Z, Z→−Y),
/// translated into the positive quadrant with the bed at z = 0.
fn print_pose() -> DAffine3 {
	DAffine3::from_mat3_translation(
		DMat3::from_cols(DVec3::X, DVec3::Z, DVec3::NEG_Y),
		v(0.0, HEIGHT, WIDTH / 2.0),
	)
}

/// Emit a shipped part: print-posed STL + 3MF with the §25 per-part gates.
fn emit_part(dir: &str, name: &str, mesh_model: &Mesh, budget_frac: f64, ok: &mut bool) -> kernel_core::mesh::SupportFreeReport {
	let m = mesh_posed(mesh_model, print_pose());
	let rep = m.support_free_report(Vec3::Z, 45.0, 0.3);
	let wt = m.is_watertight();
	let (lo, hi) = bbox(&m);
	let bed = (hi.x - lo.x).max(hi.y - lo.y) as f64;
	let budget = budget_frac * rep.total_area;
	let pass = wt && rep.steep_area <= budget.max(1e-9) && rep.max_bridge_span <= BRIDGE_MAX && bed <= BED_MAX && lo.z.abs() < 1e-3;
	*ok &= pass;
	let _ = std::fs::write(format!("{FAM}/{dir}/{name}.stl"), m.to_stl_binary());
	let _ = m.write_3mf(format!("{FAM}/{dir}/{name}.3mf"));
	println!(
		"  {name:22} wt={wt:5} steep={:8.3}/{:6.3} mm²  bridge≤{:4.1}  bed {bed:5.1}  z0 {:6.3}  {}",
		rep.steep_area,
		budget,
		rep.max_bridge_span,
		lo.z,
		if pass { "OK" } else { "<<< FAIL" }
	);
	if rep.steep_area >= 1e-6 {
		// a failing support budget must name its own offender (§25 step 3)
		for p in rep.steep_exemplars.iter().take(4) {
			println!("      steep at print ({:6.1},{:6.1},{:6.1})", p.x, p.y, p.z);
		}
	}
	rep
}

fn bbox(m: &Mesh) -> (Vec3, Vec3) {
	let mut lo = Vec3::splat(f32::INFINITY);
	let mut hi = Vec3::splat(f32::NEG_INFINITY);
	for p in &m.positions {
		lo = lo.min(*p);
		hi = hi.max(*p);
	}
	(lo, hi)
}

// ---- manifests --------------------------------------------------------------------

const FEA_DIR: &str = "bracket_system/gen_bracket/analysis/fea";

/// One selector/fixture/load block shared verbatim by every FEA job — same
/// grid frame, same clamp, same symmetry slider, same tip-pad load, so tip
/// deflections are comparable across baseline / optimized / NC.
fn fea_common(doc: &str, out_dir: &str, npy: &str) -> serde_json::Value {
	serde_json::json!({
		"_doc": doc,
		"out_dir": format!("{FEA_DIR}/{out_dir}"),
		"voxel_mm": VOX,
		"origin_mm": [0.0, 0.0, 0.0],
		"npy": format!("{FEA_DIR}/{npy}"),
		"material": {"youngs_modulus_pa": 3.5e9, "poisson": 0.36, "density_kg_m3": 1240.0},
		"fixtures": [
			{"kind": "clamped", "region_selector": {"type": "plane", "axis": "x", "value_mm": 1.3, "side": "-"}},
			{"kind": "slider", "region_selector": {"type": "plane", "axis": "y", "value_mm": 1.3, "side": "-"}}
		],
		"loads": [{"kind": "point", "magnitude": LOAD_N / 2.0, "direction": [0.0, 0.0, -1.0],
			"region_selector": {"type": "bbox", "min_mm": [PAD_X0 - 0.5, -1.0, 145.0], "max_mm": [PROJ + 0.5, 16.0, 150.5]}}]
	})
}

fn write_json(path: &str, v: &serde_json::Value) {
	let _ = std::fs::write(path, format!("{v:#}\n"));
}

// ---- main -------------------------------------------------------------------------

#[allow(clippy::too_many_lines)] // one linear, documented campaign per §25
fn main() {
	// Campaign runs always contribute to the Level-1 flywheel.
	kernel_core::telemetry::enable();
	for d in ["parts", "optional", "assembly/scene", "cad", "renders", "analysis/fea", "publish"] {
		let _ = std::fs::create_dir_all(format!("{FAM}/{d}"));
	}
	println!("BRACKET GEN — PLA shelf bracket through the full generative loop:\n");
	let mut ok = true;

	// ---- wizard pinning: the implicit screw cutters use THESE table values --------
	let m4 = metric_hole_spec(4.0).expect("M4 is in the wizard table");
	gate(
		"screw features pin to the hole-wizard M4 spec",
		(m4.countersink_d == Some(CSK_D)) && (BORE_D - m4.clearance[Fit::Medium as usize] - 0.7).abs() < 1e-9,
		format!("csk Ø{CSK_D} = DIN 74; bore Ø{BORE_D} = ISO Ø{} + 0.7 teardrop overbore", m4.clearance[1]),
		&mut ok,
	);

	// ---- stage 1: exact-B-rep baseline -------------------------------------------
	println!("\nbaseline (exact B-rep, wizard-cut M4 stations):");
	let baseline = match build_baseline() {
		Ok(s) => s,
		Err(e) => {
			gate("baseline builds", false, e, &mut ok);
			std::process::exit(1);
		}
	};
	let val = validate(&baseline);
	let m_base = tessellate_default(&baseline);
	let vol_base = volume(&baseline).abs();
	gate(
		"baseline valid + watertight",
		val.is_valid() && m_base.is_watertight(),
		format!("{:.0} mm³ = {:.0} g solid PLA", vol_base, vol_base * PLA),
		&mut ok,
	);
	let _ = std::fs::write(format!("{FEA_DIR}/baseline_analysis.stl"), m_base.to_stl_binary());
	// The baseline reaches the FEA as a SAMPLED FIELD (same path the final
	// part takes), and this gate proves that field is the same part the CAD
	// is: sampled solid volume vs the exact B-rep volume, within the
	// quantization a 2.5 mm grid can resolve on this silhouette.
	let base_field = BracketField { rho2d: unit_grid(), mutilated: false, solid_control: true, chord: false };
	let base_occ = sample_occupancy(&base_field, GRID_DIMS, Vec3::ZERO, VOX as f32);
	let _ = write_npy(&format!("{FEA_DIR}/baseline_occ.npy"), &base_occ, GRID_DIMS);
	// half model in y, so the sampled volume covers half the part
	let vol_sampled = base_occ.iter().map(|&v| v as f64).sum::<f64>() * VOX.powi(3) * 2.0;
	let dv_field = (vol_sampled - vol_base).abs() / vol_base;
	gate(
		"analysis field == the CAD baseline (sampled vs exact volume)",
		dv_field < 0.03,
		format!("{vol_sampled:.0} vs {vol_base:.0} mm³ ({:.1}%)", dv_field * 100.0),
		&mut ok,
	);
	let step_base = export_step(&baseline, "bracket_baseline");
	let _ = std::fs::write(format!("{FAM}/cad/bracket_baseline.step"), &step_base);
	match import_step(&step_base) {
		Ok(back) => {
			let dv = (volume(&back).abs() - vol_base).abs() / vol_base;
			gate("baseline STEP round-trip conserves volume (<1%)", dv < 0.01, format!("dv {:.3}%", dv * 100.0), &mut ok);
		}
		Err(e) => gate("baseline STEP round-trip", false, format!("{e:?}"), &mut ok),
	}

	// ---- stage 2: manifests + baseline FEA (receipts or die) ----------------------
	println!("\nACE loop A — baseline reference FEA (half-model, symmetry slider):");
	write_json(
		&format!("{FEA_DIR}/fea_baseline.json"),
		&fea_common(
			&format!(
				"BASELINE static case: {LOAD_KG} kg ({LOAD_N} N) down on the tip pad x>{PAD_X0}, wall face clamped, y=0 symmetry slider (half model, half load {} N). Coarse hex8 under-predicts peak bending stress ~20% (runner caveat).",
				LOAD_N / 2.0
			),
			"out_baseline",
			"baseline_occ.npy",
		),
	);
	let fea_b = require(
		"baseline FEA (tools/ace_fea_runner.py)",
		run_py("tools/ace_fea_runner.py", &format!("{FEA_DIR}/fea_baseline.json")),
		&format!("{FEA_DIR}/fea_baseline_receipt.json"),
		&mut ok,
	);
	let tip_base = f(&fea_b, &["tip_displacement_m"]) * 1000.0;
	let vm_base = f(&fea_b, &["max_von_mises_pa"]) / 1e6;
	let load_nodes = fea_b["loads"][0]["nodes_or_elements"].as_f64().unwrap_or(0.0);
	let fix_nodes = fea_b["fixtures"][0]["nodes_or_elements"].as_f64().unwrap_or(0.0);
	let broad = fea_b["notes"].as_array().map(|n| n.iter().any(|s| s.as_str().unwrap_or("").contains("suspiciously broad"))).unwrap_or(false);
	gate(
		"baseline FEA selectors honest (nodes > 0, load not smeared)",
		load_nodes > 0.0 && fix_nodes > 0.0 && !broad,
		format!("load {load_nodes:.0} / clamp {fix_nodes:.0} nodes, broad-note {broad}"),
		&mut ok,
	);
	gate(
		"baseline tip deflection sane (0.005–0.5 mm band)",
		(0.005..=0.5).contains(&tip_base),
		format!("tip {tip_base:.4} mm, peak vm {vm_base:.2} MPa"),
		&mut ok,
	);

	// ---- stage 3: SIMP optimize, TWICE (determinism receipt) ----------------------
	println!("\nACE loop B — SIMP topology optimization ({SIMP_MAX_ITERS} iters cap, volfrac {VOLFRAC}):");
	let opt_doc = format!(
		"SIMP on the baseline occupancy. Frozen = wall plate (x<{PLATE_T}) and shelf strip (z>{}); the two driver tunnels are VOID keep-outs so material is routed around them. Frozen still re-pins the countersink voxels inside the plate to 1.0 (kind precedence) — those holes are re-cut in the rebuild and the FINAL binary FEA of the drilled part is the gate. volfrac {VOLFRAC} on the design region.",
		HEIGHT - ARM_T
	);
	let mk_opt = |out: &str| {
		let mut j = fea_common(&opt_doc, out, "baseline_occ.npy");
		let o = j.as_object_mut().unwrap();
		o.insert("volfrac".into(), serde_json::json!(VOLFRAC));
		o.insert("penalty".into(), serde_json::json!(SIMP_PENALTY));
		o.insert("filter_radius_vox".into(), serde_json::json!(SIMP_FILTER_RVOX));
		o.insert("max_iters".into(), serde_json::json!(SIMP_MAX_ITERS));
		o.insert("move".into(), serde_json::json!(0.2));
		o.insert("iso".into(), serde_json::json!(ISO));
		o.insert("time_budget_s".into(), serde_json::json!(600.0));
		// Keep-outs are the REAL geometry, not a sketch of it. The frozen set
		// is the wall plate (screw bearing + the clamped face) and the shelf
		// strip (the load pad's seat); the driver tunnels are declared VOID so
		// the optimizer routes material AROUND the holes the rebuild will
		// actually bore. Earlier revisions froze two screw-boss bands instead
		// and never mentioned the tunnels — SIMP then treated the tunnel cores as
		// load-bearing and spent stiffness the finished part does not have.
		let tunnel_span = |zs: f64| {
			let x1 = hyp_x(zs + TUNNEL_D / 2.0) + 5.0;
			((PLATE_T + x1) / 2.0, x1 - PLATE_T)
		};
		let (tc_top, tl_top) = tunnel_span(SCREW_TOP_Z);
		let (tc_bot, tl_bot) = tunnel_span(SCREW_BOT_Z);
		o.insert(
			"regions".into(),
			serde_json::json!([
				{"kind": "frozen", "selector": {"type": "bbox", "min_mm": [-1.0, -1.0, -1.0], "max_mm": [PLATE_T, 16.0, HEIGHT + 1.0]}},
				{"kind": "frozen", "selector": {"type": "bbox", "min_mm": [-1.0, -1.0, HEIGHT - ARM_T], "max_mm": [PROJ + 1.0, 16.0, HEIGHT + 1.0]}},
				{"kind": "void", "selector": {"type": "cylinder", "axis": "x", "center_mm": [tc_top, 0.0, SCREW_TOP_Z], "radius_mm": TUNNEL_D / 2.0, "length_mm": tl_top}},
				{"kind": "void", "selector": {"type": "cylinder", "axis": "x", "center_mm": [tc_bot, 0.0, SCREW_BOT_Z], "radius_mm": TUNNEL_D / 2.0, "length_mm": tl_bot}}
			]),
		);
		j
	};
	write_json(&format!("{FEA_DIR}/opt_a.json"), &mk_opt("out_opt_a"));
	write_json(&format!("{FEA_DIR}/opt_b.json"), &mk_opt("out_opt_b"));
	let opt_a = require(
		"SIMP run A (tools/ace_optimize_runner.py)",
		run_py("tools/ace_optimize_runner.py", &format!("{FEA_DIR}/opt_a.json")),
		&format!("{FEA_DIR}/opt_a_receipt.json"),
		&mut ok,
	);
	let opt_b = require(
		"SIMP run B (same manifest, fresh out_dir)",
		run_py("tools/ace_optimize_runner.py", &format!("{FEA_DIR}/opt_b.json")),
		&format!("{FEA_DIR}/opt_b_receipt.json"),
		&mut ok,
	);
	// ACE's gated-STL step writes a scratch job JSON per meshing attempt into
	// the out_dir; they are not receipts and they accumulate every run.
	for dir in ["out_opt_a", "out_opt_b"] {
		if let Ok(entries) = std::fs::read_dir(format!("{FEA_DIR}/{dir}")) {
			for e in entries.flatten() {
				let name = e.file_name().to_string_lossy().to_string();
				if name.starts_with("tmp") && name.ends_with(".json") || name == "_lmcad_rho.npy" {
					let _ = std::fs::remove_file(e.path());
				}
			}
		}
	}
	let rho_a = std::fs::read(format!("{FEA_DIR}/out_opt_a/final_rho.npy")).unwrap_or_default();
	let rho_b = std::fs::read(format!("{FEA_DIR}/out_opt_b/final_rho.npy")).unwrap_or_default();
	gate(
		"SIMP deterministic: two runs, byte-identical final_rho.npy",
		!rho_a.is_empty() && rho_a == rho_b && f(&opt_a, &["compliance_last"]) == f(&opt_b, &["compliance_last"]),
		format!("{} bytes, compliance {:.6e}", rho_a.len(), f(&opt_a, &["compliance_last"])),
		&mut ok,
	);
	let vf = f(&opt_a, &["volume_fraction_achieved"]);
	gate(
		"SIMP volume constraint held (≤ volfrac + 0.02)",
		vf <= VOLFRAC + 0.02,
		format!("achieved {vf:.4} vs target {VOLFRAC}, {} iters, stop: {}", f(&opt_a, &["iterations"]), opt_a["stop_reason"].as_str().unwrap_or("?")),
		&mut ok,
	);
	gate(
		"SIMP stiffened vs iteration 1 (compliance monotone trend)",
		f(&opt_a, &["compliance_last"]) < f(&opt_a, &["compliance_first"]),
		format!("{:.4e} -> {:.4e}", f(&opt_a, &["compliance_first"]), f(&opt_a, &["compliance_last"])),
		&mut ok,
	);
	gate(
		"SIMP gated STL emitted watertight (kernel meshing pipeline)",
		opt_a["stl"]["ok"].as_bool().unwrap_or(false) && opt_a["stl"]["watertight"].as_bool().unwrap_or(false),
		format!("upsample x{}, {} tris", f(&opt_a, &["stl", "mesh_upsample"]), f(&opt_a, &["stl", "num_triangles"])),
		&mut ok,
	);

	// ---- stage 4: density → GridField → implicit body -----------------------------
	println!("\nrebuild — density field to implicit to exact:");
	// element centers sit half a cell above the node origin (grid_field.rs doc)
	let half = GridField::from_npy_file(
		format!("{FEA_DIR}/out_opt_a/final_rho.npy"),
		Vec3::new((VOX / 2.0) as f32, (VOX / 2.0) as f32, (VOX / 2.0) as f32),
		VOX as f32,
	)
	.unwrap_or_else(|e| {
		gate("final_rho.npy loads as a GridField", false, e, &mut ok);
		std::process::exit(1);
	});
	// y-average (manufacturing regularization: the shipped web is a 2.5D
	// extrusion — that is what makes the side-lying print support-free), then
	// one 3×3 tent blur in (x, z) — the "smooth" of threshold+smooth.
	let (nx, ny, nz) = half.dims();
	let mut rho2d = vec![0.0f32; nx * nz];
	for i in 0..nx {
		for k in 0..nz {
			let mut acc = 0.0f32;
			for j in 0..ny {
				acc += half.sample(Vec3::new(
					(VOX / 2.0 + VOX * i as f64) as f32,
					(VOX / 2.0 + VOX * j as f64) as f32,
					(VOX / 2.0 + VOX * k as f64) as f32,
				));
			}
			rho2d[i * nz + k] = acc / ny as f32;
		}
	}
	let blur = |src: &[f32]| -> Vec<f32> {
		let mut out = vec![0.0f32; nx * nz];
		let at = |i: i64, k: i64| src[(i.clamp(0, nx as i64 - 1) as usize) * nz + k.clamp(0, nz as i64 - 1) as usize];
		for i in 0..nx as i64 {
			for k in 0..nz as i64 {
				let mut acc = 0.0f32;
				for (di, wi) in [(-1i64, 0.25f32), (0, 0.5), (1, 0.25)] {
					for (dk, wk) in [(-1i64, 0.25f32), (0, 0.5), (1, 0.25)] {
						acc += wi * wk * at(i + di, k + dk);
					}
				}
				out[i as usize * nz + k as usize] = acc;
			}
		}
		out
	};
	let rho2d = blur(&rho2d);
	let grid2d = GridField::from_data(rho2d, (nx, 1, nz), Vec3::new((VOX / 2.0) as f32, 0.0, (VOX / 2.0) as f32), VOX as f32)
		.expect("2.5D density grid is finite by construction");
	let field = BracketField { rho2d: grid2d, mutilated: false, solid_control: false, chord: true };

	// Minimum length scale is IMPOSED by the density filter, not hoped for:
	// the top88 cone filter blurs over its radius, so no member survives
	// thinner than ~2·r. That is the checkable, non-gameable statement about
	// the optimizer's setup (the probe below then measures the result).
	gate(
		"SIMP filter imposes a printable length scale (2·r ≥ 1.6 mm)",
		2.0 * SIMP_FILTER_RVOX * VOX >= MIN_FEATURE as f64,
		format!("2·{SIMP_FILTER_RVOX}·{VOX} = {:.1} mm ≥ {MIN_FEATURE}", 2.0 * SIMP_FILTER_RVOX * VOX),
		&mut ok,
	);
	// thin-wall probe on the final implicit body (sampled estimate — can
	// under-report by ~one cell) AND on the SOLID control: same silhouette,
	// same chord, same screw cuts, density ignored. The control is what
	// separates a floor set by the SILHOUETTE from one the optimizer
	// introduced; both numbers are reported, neither is massaged.
	let control = BracketField { solid_control: true, ..field.clone() };
	let tw = reverse::thin_wall_report(&field, field.bounds(), 110, MIN_FEATURE);
	let tw_ctl = reverse::thin_wall_report(&control, control.bounds(), 110, MIN_FEATURE);
	// MEASURED 2026-07-31: the SOLID control reports the same 0.03 mm
	// `thinnest` as the optimized body. An absolute "thinnest ≥ 1.6" gate is
	// therefore blind here — it fails a part that is by definition printable
	// (a solid triangle), because `thinnest` is set by TAPERS: where a Ø14
	// driver tunnel grazes the sloped silhouette, material wedges to zero and
	// a finer probe just finds a smaller number. A taper is not a thin wall;
	// the slicer drops sub-width perimeters at the tip. So the gate asks the
	// question that DOES discriminate — did the generative step introduce
	// thin material the solid part does not already have? — on both the
	// minimum and the census. The thinnest MEMBER (as opposed to taper) is
	// measured out-of-band on the 2.5D section: 3.50 mm, no sample under
	// 1.6 mm, which is what the SIMP filter-radius gate above buys.
	// What this gate proves, exactly: the optimized body's thinnest reading is
	// indistinguishable from the SOLID control's at the probe's own stated
	// resolution (it "can under-report by up to ~one lattice cell" —
	// thin_wall_report's doc). Both read ≈0 because both are TAPER-limited,
	// and a finer probe would only return a smaller number on either.
	// What it does NOT prove: that no member is thin. That question is
	// answered by the filter-length-scale gate above and, out of band, by a
	// medial-axis measurement of the rebuilt 2.5D section — thinnest member
	// 3.50 mm, zero samples under 1.6 mm (recorded in DESIGN.md). The
	// sub-threshold COUNTS are reported here rather than gated: a count is a
	// function of how much tapered edge the topology has, not of whether any
	// of it is unprintable, and inventing a ratio to bless it would be
	// exactly the kind of number this campaign refuses to ship.
	let probe_cell = (field.bounds().size().max_element()) / 109.0;
	gate(
		"thin wall: optimized indistinguishable from the solid control",
		(tw.thinnest - tw_ctl.thinnest).abs() <= probe_cell,
		format!(
			"opt {:.2} mm/{} vs control {:.2} mm/{} (probe cell {probe_cell:.2})",
			tw.thinnest, tw.below_count, tw_ctl.thinnest, tw_ctl.below_count
		),
		&mut ok,
	);
	// NC: the probe must FIRE on a deliberately thin body (0.8 mm plate)
	let thin_probe = reverse::thin_wall_report(
		&kernel_implicit::Cuboid::new(Vec3::new(50.0, 0.0, 75.0), Vec3::new(50.0, 0.4, 75.0)),
		Aabb::new(Vec3::new(-1.0, -2.0, -1.0), Vec3::new(101.0, 2.0, 151.0)),
		64,
		MIN_FEATURE,
	);
	gate(
		"NC: thin-wall oracle fires on a 0.8 mm slab",
		thin_probe.thinnest < 1.0 && thin_probe.below_count > 0,
		format!("thinnest {:.2}, {} below", thin_probe.thinnest, thin_probe.below_count),
		&mut ok,
	);

	// ---- back to exact: reverse bridge (recovered, else v1 + stated why) ----------
	// The MESHER's own output is the print/analysis artifact: it is watertight
	// by construction. The reverse bridge exists to make the same geometry
	// exact for STEP — it is not in the path to the STL. That split is not a
	// convenience: `mesh_to_solid` coalesces adjacent facets into multi-loop
	// planar faces (genus 5 here, from the four screw bores), and
	// re-tessellating THOSE does not come back watertight. Shipping that
	// re-tessellation fed a leaky mesh to `voxelize_stl.py`, whose parity fill
	// then reported a 4× tip deflection and an NC that read stiffer than the
	// real part. Same geometry, one fewer lossy round-trip.
	let domain = field.bounds().pad(MESH_VOX * 2.0);
	let m_opt = kernel_implicit::manifold_dual_contour(&field, domain, kernel_core::mesher::Resolution::VoxelSize(MESH_VOX));
	gate(
		"optimized mesh is watertight (the print/FEA artifact)",
		m_opt.is_watertight() && m_opt.triangle_count() > 0,
		format!("{} tris, {} non-manifold edges", m_opt.triangle_count(), m_opt.non_manifold_edge_count()),
		&mut ok,
	);
	// v1 first: it is the route whose contract covers a faceted organic web.
	let faceted = match reverse::mesh_to_solid(&m_opt) {
		Ok(s) => s,
		Err(e) => {
			gate("reverse bridge (v1 faceted)", false, e, &mut ok);
			std::process::exit(1);
		}
	};
	// v2 recovery is a FINISHING pass and must earn its keep: it is kept only
	// if it passes the SAME validity + watertight gates the shipped part is
	// held to. Measured 2026-07-30: recovery fitted 6 cylinder carriers at
	// residual 0.0757 mm (tol 0.08) and the resulting solid tessellated
	// NON-watertight — the sector-merge/boundary-ring limit named in the
	// reverse module doc. Silently shipping it fed a leaky STL to the
	// voxelizer, whose parity fill then reported a 143× tip deflection: the
	// route choice is a structural correctness question, not cosmetics.
	// v2 is accepted only if it is valid AND still conserves volume against
	// the shipped mesh to the v1 bridge's own 1e-6 — `recover_quadrics`
	// permits up to 0.5% drift, and cad/ must be the part parts/ is.
	let conserves = |s: &Solid| {
		let vm = m_opt.signed_volume().abs();
		(volume(s).abs() - vm).abs() <= 1e-6 * vm.max(1.0)
	};
	let (optimized, route) = match reverse::mesh_to_solid_recovered(&m_opt, 0.08) {
		Ok((s, rep)) if validate(&s).is_valid() && conserves(&s) => {
			let r = format!(
				"v2 recovered: {} cyl / {} cone / {} sphere / {} torus, residual {:.4} mm",
				rep.cylinders, rep.cones, rep.spheres, rep.tori, rep.max_fit_residual
			);
			(s, r)
		}
		Ok((s, rep)) => {
			let vm = m_opt.signed_volume().abs();
			let why = format!(
				"v1 faceted — v2 recovery REJECTED (fitted {} cyl at residual {:.4} mm; valid={} vol drift {:.2e})",
				rep.cylinders,
				rep.max_fit_residual,
				validate(&s).is_valid(),
				(volume(&s).abs() - vm).abs() / vm
			);
			println!("  note: {why}");
			(faceted, why)
		}
		Err(e) => {
			let why = format!("v1 faceted — v2 recovery refused: {}", e.chars().take(90).collect::<String>());
			println!("  note: {why}");
			(faceted, why)
		}
	};
	let val_o = validate(&optimized);
	let vol_opt = volume(&optimized).abs();
	// The bridge's own conservation gate already refuses a wrap that changed
	// geometry (1e-6 relative); this asserts the exact solid the STEP carries
	// agrees with the mesh the printer gets, so cad/ and parts/ are one part.
	let vol_mesh = m_opt.signed_volume().abs();
	gate(
		"optimized bridges to a valid exact solid (STEP route)",
		val_o.is_valid() && (vol_opt - vol_mesh).abs() <= 1e-6 * vol_mesh.max(1.0),
		format!("{} faces, {route}", optimized.face_count()),
		&mut ok,
	);
	// One connected body: a floating island would voxelize into a load path
	// that does not exist (and would print as loose debris).
	gate(
		"optimized is ONE connected shell (no floating islands)",
		val_o.shells == 1,
		format!("shells {}, genus {}", val_o.shells, val_o.genus),
		&mut ok,
	);
	let step_opt = export_step(&optimized, "bracket_optimized");
	let _ = std::fs::write(format!("{FAM}/cad/bracket_optimized.step"), &step_opt);
	gate(
		"optimized STEP written",
		step_opt.len() > 1000,
		format!("{:.1} MB", step_opt.len() as f64 / 1e6),
		&mut ok,
	);

	// ---- stage 5: HONEST re-analysis of the final binary geometry -----------------
	println!("\nACE loop C — honest re-analysis of the FINAL geometry:");
	let _ = std::fs::write(format!("{FEA_DIR}/optimized_analysis.stl"), m_opt.to_stl_binary());
	let final_occ = sample_occupancy(&field, GRID_DIMS, Vec3::ZERO, VOX as f32);
	let _ = write_npy(&format!("{FEA_DIR}/final_occ.npy"), &final_occ, GRID_DIMS);
	// the sampled analysis body and the SHIPPED mesh must be the same part
	let vol_occ = final_occ.iter().map(|&v| v as f64).sum::<f64>() * VOX.powi(3) * 2.0;
	let dv_occ = (vol_occ - vol_mesh).abs() / vol_mesh;
	gate(
		"analysis field == the shipped mesh (sampled vs meshed volume)",
		dv_occ < 0.05,
		format!("{vol_occ:.0} vs {vol_mesh:.0} mm³ ({:.1}%)", dv_occ * 100.0),
		&mut ok,
	);
	write_json(
		&format!("{FEA_DIR}/fea_final.json"),
		&fea_common(
			"FINAL as-built case: identical grid/fixtures/load to the baseline job — the honest-re-analysis doctrine (gate the printed geometry, not the SIMP proxy).",
			"out_final",
			"final_occ.npy",
		),
	);
	let fea_f = require(
		"final FEA (tools/ace_fea_runner.py)",
		run_py("tools/ace_fea_runner.py", &format!("{FEA_DIR}/fea_final.json")),
		&format!("{FEA_DIR}/fea_final_receipt.json"),
		&mut ok,
	);
	let tip_final = f(&fea_f, &["tip_displacement_m"]) * 1000.0;
	let vm_final = f(&fea_f, &["max_von_mises_pa"]) / 1e6;
	let mass_red = 1.0 - vol_opt / vol_base;
	gate(
		&format!("mass reduction ≥ {:.0}%", MASS_RED_MIN * 100.0),
		mass_red >= MASS_RED_MIN,
		format!("{:.0} g -> {:.0} g ({:.1}% cut)", vol_base * PLA, vol_opt * PLA, mass_red * 100.0),
		&mut ok,
	);
	gate(
		&format!("tip deflection ≤ {STIFF_FACTOR}× baseline"),
		tip_final <= STIFF_FACTOR * tip_base && tip_final.is_finite(),
		format!("{tip_final:.4} vs baseline {tip_base:.4} mm ({:.2}×)", tip_final / tip_base),
		&mut ok,
	);
	let sig_rt = materials::pla::SIG_ALLOW_RT;
	let sig_hot = materials::pla::SIG_ALLOW_HOT;
	let vm_design = vm_final * HEX8_PEAK_FACTOR;
	gate(
		"transient peak stress ×1.25 hex8 derate ≤ PLA RT allowable",
		vm_design <= sig_rt,
		format!("{vm_final:.2} MPa ×{HEX8_PEAK_FACTOR} = {vm_design:.2} vs {sig_rt} MPa"),
		&mut ok,
	);
	// THE GOVERNING CHECK. A shelf bracket holds its load continuously, so
	// the short-term allowable above answers the wrong question (someone
	// leaning on the shelf). The sustained allowable comes from the
	// researched creep table, read from the material record so the number
	// cannot drift from its provenance.
	let sig_creep = materials::pla::creep_allowable_mpa(CREEP_DESIGN_T_C, CREEP_DESIGN_HOURS);
	gate(
		"SUSTAINED stress ≤ creep allowable 23 °C / 1 y (GOVERNING)",
		vm_design <= sig_creep && sig_creep > 0.0,
		format!("{vm_design:.2} vs {sig_creep} MPa (margin ×{:.2})", sig_creep / vm_design),
		&mut ok,
	);
	// Fastener seat — closed-form, and gated against the SAME sustained
	// allowable as the web, because the screw holds its tension for years
	// too. Conservative on every term: the lever is the FULL projection (the
	// load can sit at the very tip), the couple is reacted by the screw pair
	// alone (no credit for the plate bearing on the wall below the bottom
	// screw, which would lengthen the arm), and the seat area is discounted
	// by SEAT_BEARING_FRAC for the crown-roof cap.
	let screw_tension = LOAD_N * PROJ / (SCREW_TOP_Z - SCREW_BOT_Z);
	let seat_area = |od: f64| std::f64::consts::PI / 4.0 * (od * od - BORE_D * BORE_D) * SEAT_BEARING_FRAC;
	let seat_mpa = screw_tension / seat_area(WASHER_OD);
	let bare_head_mpa = screw_tension / seat_area(8.0); // DIN 7991 M4 head Ø8
	gate(
		"screw seat bearing ≤ sustained allowable (WITH the Ø10 washer)",
		seat_mpa <= sig_creep,
		format!("{seat_mpa:.2} vs {sig_creep} MPa (bare M4 head would be {bare_head_mpa:.2} — hence the washer)"),
		&mut ok,
	);
	// The side-lying print is what keeps that creep number applicable: the
	// bending stress runs IN the layer plane, so the record's across-layer
	// knockdown (0.55) does NOT apply. Printed upright it would.
	let build_dir_model = DVec3::Y; // print +Z is model +Y (see print_pose)
	let stress_dir_model = DVec3::new(1.0, 0.0, 1.0).normalize(); // web bending, X–Z plane
	let out_of_plane_deg = stress_dir_model.dot(build_dir_model).abs().asin().to_degrees();
	gate(
		"load path lies IN the layer plane (no 0.55 across-layer derate)",
		out_of_plane_deg < 30.0,
		format!("{out_of_plane_deg:.1}° out of plane vs 30° threshold"),
		&mut ok,
	);

	// NC: the FEA pipeline must SEE geometry — a mutilated bracket (lower web
	// chopped) must deflect far more through the SAME manifest chain.
	println!("\nACE loop D — negative control (mutilated geometry through the same chain):");
	let nc_field = BracketField { mutilated: true, ..field.clone() };
	let nc_occ = sample_occupancy(&nc_field, GRID_DIMS, Vec3::ZERO, VOX as f32);
	let _ = write_npy(&format!("{FEA_DIR}/nc_occ.npy"), &nc_occ, GRID_DIMS);
	let nc_solid: f64 = nc_occ.iter().map(|&v| v as f64).sum();
	let fin_solid: f64 = final_occ.iter().map(|&v| v as f64).sum();
	gate(
		"NC geometry really is mutilated (material removed)",
		nc_solid < 0.9 * fin_solid,
		format!("{nc_solid:.0} vs {fin_solid:.0} solid-fraction voxels"),
		&mut ok,
	);
	write_json(
		&format!("{FEA_DIR}/fea_nc.json"),
		&fea_common("NC: identical manifest to the final case on deliberately broken geometry — tip deflection must JUMP.", "out_nc", "nc_occ.npy"),
	);
	let fea_nc = require(
		"NC FEA (tools/ace_fea_runner.py)",
		run_py("tools/ace_fea_runner.py", &format!("{FEA_DIR}/fea_nc.json")),
		&format!("{FEA_DIR}/fea_nc_receipt.json"),
		&mut ok,
	);
	let tip_nc = f(&fea_nc, &["tip_displacement_m"]) * 1000.0;
	gate(
		"NC: FEA fires on the mutilated bracket (≥ 1.5× final tip)",
		tip_nc >= 1.5 * tip_final,
		format!("NC tip {tip_nc:.4} vs final {tip_final:.4} mm ({:.1}×)", tip_nc / tip_final),
		&mut ok,
	);

	// ---- stage 5b: buckling of the web's compression diagonals --------------------
	// A topology-optimized web IS a truss: its lower diagonal runs in
	// compression, which is exactly where elastic buckling can beat strength.
	// The mode is genuine here (not plan padding), so it gets receipts.
	println!("\nACE loop E — linear buckling of the optimized web (compression diagonals):");
	let mut buckle_job = fea_common(
		"Buckling of the FINAL geometry under the rated reference load, same grid/fixtures/load as the static case. Linear bifurcation on perfect geometry = UPPER bound; the gate uses the runner's cited 0.5 FDM knockdown.",
		"out_buckle",
		"final_occ.npy",
	);
	if let Some(o) = buckle_job.as_object_mut() {
		o.insert("n_modes".into(), serde_json::json!(4));
		o.insert("knockdown".into(), serde_json::json!(0.5));
	}
	write_json(&format!("{FEA_DIR}/buckle_final.json"), &buckle_job);
	let buckle = require(
		"buckling (tools/ace_buckling_runner.py)",
		run_py("tools/ace_buckling_runner.py", &format!("{FEA_DIR}/buckle_final.json")),
		&format!("{FEA_DIR}/buckle_final_receipt.json"),
		&mut ok,
	);
	let lam = f(&buckle, &["buckling_load_factor"]);
	let design_crit = f(&buckle, &["knockdown", "design_critical_load_n"]);
	let applied_ref = f(&buckle, &["applied_reference_load_N"]);
	let buckle_margin = design_crit / applied_ref;
	gate(
		&format!("buckling not governing: knocked-down critical ≥ {BUCKLE_MIN_FACTOR}× rated"),
		buckle_margin >= BUCKLE_MIN_FACTOR,
		format!("λ {lam:.1}, design crit {design_crit:.0} N vs applied {applied_ref:.1} N (×{buckle_margin:.1})"),
		&mut ok,
	);

	// ---- stage 6: print-readiness + shipped parts ---------------------------------
	println!("\nprint audit (side-lying pose, +Y up in model frame):");
	let rep_final = emit_part("parts", "bracket_optimized", &m_opt, SUPPORT_BUDGET_FRAC, &mut ok);
	// The budget only means something if the residual is WHERE it is claimed
	// to be. Every flagged patch must sit within SCREW_FEATURE_R of a screw
	// axis (the teardrop apex ridges) — i.e. the optimized WEB, the part the
	// generative step actually produced, audits clean. Print pose maps model
	// (x, y, z) -> (x, HEIGHT - z, y + WIDTH/2), so invert it to measure.
	let worst_r = rep_final
		.steep_exemplars
		.iter()
		.map(|p| {
			let (my, mz) = (p.z as f64 - WIDTH / 2.0, HEIGHT - p.y as f64);
			[SCREW_TOP_Z, SCREW_BOT_Z]
				.iter()
				.map(|&zs| (my * my + (mz - zs) * (mz - zs)).sqrt())
				.fold(f64::INFINITY, f64::min)
		})
		.fold(0.0, f64::max);
	gate(
		"support residual confined to the screw roofs (web audits clean)",
		rep_final.steep_exemplars.is_empty() || worst_r <= SCREW_FEATURE_R,
		format!("worst exemplar {worst_r:.1} mm from a screw axis (≤ {SCREW_FEATURE_R})"),
		&mut ok,
	);
	let coupon = match build_coupon() {
		Ok(s) => s,
		Err(e) => {
			gate("coupon builds", false, e, &mut ok);
			std::process::exit(1);
		}
	};
	let m_coupon = tessellate_default(&coupon);
	// the exact B-rep coupon gets NO budget: it must audit dead clean
	let _ = emit_part("optional", "coupon_m4_seat", &m_coupon, 0.0, &mut ok);
	let _ = std::fs::write(format!("{FAM}/cad/coupon_m4_seat.step"), export_step(&coupon, "coupon_m4_seat"));
	// NC: the SAME audit must fire in a wrong orientation (installed pose:
	// the shelf strip's underside becomes a raw horizontal overhang)
	let rep_wrong = m_opt.support_free_report(Vec3::Z, 45.0, 0.3);
	gate(
		"NC: audit fires on the upright (as-installed) orientation",
		rep_wrong.steep_area > 1000.0,
		format!("steep {:.0} mm²", rep_wrong.steep_area),
		&mut ok,
	);

	// ---- stage 7: deliverables ----------------------------------------------------
	println!("\ndeliverables:");
	// assembly scene: bracket upright + wall + shelf + screws (mocks)
	let wall = prism_y(&[(-12.0, -25.0), (0.0, -25.0), (0.0, 175.0), (-12.0, 175.0)], -80.0, 80.0);
	let shelf = prism_y(&[(0.0, HEIGHT), (118.0, HEIGHT), (118.0, HEIGHT + 18.0), (0.0, HEIGHT + 18.0)], -80.0, 80.0);
	let screw = |zs: f64| -> Solid {
		let shank = kernel_brep::cylinder(v(PLATE_T - 1.0, 0.0, zs), -DVec3::X, 2.0, 24.0, 24);
		let head = kernel_brep::cone(v(PLATE_T - 2.75, 0.0, zs), DVec3::X, 2.25, 5.0, 24);
		union(&shank, &head)
	};
	let mut scene = Mesh::default();
	merge_into(&mut scene, &m_opt);
	merge_into(&mut scene, &tessellate_default(&wall));
	merge_into(&mut scene, &tessellate_default(&shelf));
	merge_into(&mut scene, &tessellate_default(&screw(SCREW_TOP_Z)));
	merge_into(&mut scene, &tessellate_default(&screw(SCREW_BOT_Z)));
	let _ = std::fs::write(format!("{FAM}/assembly/assembly.stl"), scene.to_stl_binary());
	let _ = std::fs::write(format!("{FAM}/assembly/scene/bracket.stl"), m_opt.to_stl_binary());
	let _ = std::fs::write(format!("{FAM}/assembly/scene/wall_mock.stl"), tessellate_default(&wall).to_stl_binary());
	let _ = std::fs::write(format!("{FAM}/assembly/scene/shelf_mock.stl"), tessellate_default(&shelf).to_stl_binary());
	let mut screws = Mesh::default();
	merge_into(&mut screws, &tessellate_default(&screw(SCREW_TOP_Z)));
	merge_into(&mut screws, &tessellate_default(&screw(SCREW_BOT_Z)));
	let _ = std::fs::write(format!("{FAM}/assembly/scene/screws_mock.stl"), screws.to_stl_binary());
	let _ = std::fs::write(format!("{FAM}/assembly/scene/baseline_ref.stl"), m_base.to_stl_binary());

	// assembly sheet + instructions via assembly_doc.py (deterministic date)
	write_json(
		&format!("{FAM}/assembly/scene/sheet_job.json"),
		&serde_json::json!({
			"parts": [
				{"name": "wall (yours)", "stl": format!("{FAM}/assembly/scene/wall_mock.stl"), "color": "lightgray"},
				{"name": "bracket_optimized", "stl": format!("{FAM}/assembly/scene/bracket.stl"), "color": "steelblue"},
				{"name": "M4 screws", "stl": format!("{FAM}/assembly/scene/screws_mock.stl"), "color": "dimgray"},
				{"name": "shelf (yours)", "stl": format!("{FAM}/assembly/scene/shelf_mock.stl"), "color": "burlywood"}
			],
			"explode": {"axis": [1.0, 0.0, 0.0], "auto": true, "gap_mm": 14},
			"steps": [
				{"order": 1, "text": "Print optional/coupon_m4_seat first; an M4 flat-head must pull flush into the countersink. Adjust flow/XY if proud.", "fasteners": "1x M4x12 test screw"},
				{"order": 2, "text": "Level the bracket on the wall, mark through both bores, drill Ø6 and fit wall plugs (or drive wood screws into a stud)."},
				{"order": 3, "text": "Drop an M4 cup washer into each countersink FIRST (required — it spreads the head load over the printed cone), then screw the bracket on through both teardrop tunnels until the heads seat flush.", "fasteners": "2x M4x30 DIN 7991 + 2x M4 Ø10 cup washer + Ø6 plugs"},
				{"order": 4, "text": "Set the shelf on the 100 mm arm. Rated 10 kg per bracket at the tip; use two brackets per shelf."}
			],
			"out_prefix": format!("{FAM}/assembly/bracket"),
			"date": "2026-07-30",
			"project": "LMCAD bracket_system",
			"doc_title": "gen_bracket — wall mounting"
		}),
	);
	let sheet = run_py("tools/assembly_doc.py", &format!("{FAM}/assembly/scene/sheet_job.json"));
	match sheet {
		Ok(_) => {
			let _ = std::fs::rename(format!("{FAM}/assembly/bracket_assembly_doc.png"), format!("{FAM}/assembly/ASSEMBLY.png"));
			let _ = std::fs::rename(format!("{FAM}/assembly/bracket_instructions.md"), format!("{FAM}/assembly/instructions.md"));
			gate("assembly sheet rendered (ASSEMBLY.png)", true, "assembly_doc.py".to_string(), &mut ok);
		}
		Err(e) => gate("assembly sheet rendered (ASSEMBLY.png)", false, e.chars().take(110).collect(), &mut ok),
	}

	// renders (render_views.py is matplotlib-only; treat as required deliverable)
	let r1 = run_py_plain("tools/render_views.py", &[&format!("{FAM}/assembly/scene/bracket.stl"), &format!("{FAM}/renders/render_bracket.png")]);
	let r2 = run_py_plain("tools/render_views.py", &[&format!("{FAM}/assembly/scene/baseline_ref.stl"), &format!("{FAM}/renders/render_baseline.png")]);
	let r3 = run_py_plain("tools/render_views.py", &[&format!("{FAM}/assembly/assembly.stl"), &format!("{FAM}/renders/render_assembly.png")]);
	gate(
		"renders written (bracket, baseline, assembly)",
		r1.is_ok() && r2.is_ok() && r3.is_ok(),
		format!("{}", [&r1, &r2, &r3].iter().filter(|r| r.is_ok()).count()),
		&mut ok,
	);

	// BOM
	let bom = format!(
		"# GEN BRACKET — bill of materials (per bracket)\n\n| item | qty | source | notes |\n|---|---|---|---|\n| bracket_optimized (parts/) | 1 | print | PLA, {br:.0} g solid-equivalent, side-lying, no supports |\n| M4×30 countersunk screw, DIN 7991 | 2 | purchased | flat head seats in the DIN 74 countersink |\n| **M4 countersunk (cup) washer, Ø{wod:.0} OD** | **2** | purchased | **REQUIRED, not optional**: it spreads the head load over the printed cone. Without it the seat bearing ({bare:.2} MPa) exceeds PLA's sustained creep allowable ({sc} MPa); with it, {seat:.2} MPa |\n| Ø6 wall plug (or wood screw into stud) | 2 | purchased | anchor tension ≈ {tens:.0} N at rated load — any plug class covers it |\n| coupon_m4_seat (optional/) | 1 | print (pre-flight) | {cp:.0} g, 15-minute seat/roof check |\n\nUse two brackets per shelf. No supports, no inserts, no glue.\n",
		br = vol_opt * PLA,
		wod = WASHER_OD,
		bare = bare_head_mpa,
		sc = sig_creep,
		seat = seat_mpa,
		tens = screw_tension,
		cp = volume(&coupon).abs() * PLA,
	);
	let _ = std::fs::write(format!("{FAM}/assembly/BOM.md"), bom);

	// generated ANALYSIS.md — every number from THIS run
	let analysis = format!(
		r#"# GEN BRACKET — analysis (generated by bracket_gen.rs; regenerated every run)

The full generative loop ran to produce these numbers — job manifests and
receipts in `fea/` (regenerate: `sh fea/run_opt.sh`). Loop narrative and
route choices: DESIGN.md.

## The loop, measured

| stage | mass (solid PLA) | tip deflection @ {LOAD_N:.1} N | peak von Mises |
|---|---|---|---|
| baseline (solid triangle) | {mb:.0} g | {tb:.4} mm | {vb:.2} MPa |
| SIMP optimized, as rebuilt | {mo:.0} g | {tf:.4} mm | {vf_:.2} MPa |
| change | **−{mr:.1}%** | ×{ratio:.2} | ×{vr:.2} |

- SIMP: {it:.0} iterations (stop: {stop}), compliance {c0:.3e} → {c1:.3e},
  volume fraction achieved {vfa:.3} (target {VOLFRAC}) — receipts
  `fea/opt_a_receipt.json` (+ byte-identical run B: determinism gate).
- FEA: ACE hex8 linear-elastic, voxel {VOX} mm, half model with a y-symmetry
  slider; identical fixtures/load selectors for every case, so the tips are
  comparable. Coarse hex8 under-predicts peak bending stress ~20% (runner's
  caveat) — the RT gate derates the peak by ×{HEX8_PEAK_FACTOR}.

## Stress vs printed-PLA allowables

Peak von Mises on the final geometry is {vf_:.2} MPa; every row compares the
hex8-derated **{vm_d:.2} MPa** ({vf_:.2} × {HEX8_PEAK_FACTOR}).

| tier | allowable | source | margin |
|---|---|---|---|
| **SUSTAINED 23 °C / 1 y — GOVERNING** | **{sig_creep} MPa** | `materials::pla::creep_allowable_mpa` | **×{mcreep:.2}** |
| transient RT (20 °C), σ | {sig_rt} MPa | `materials::pla::SIG_ALLOW_RT` | ×{mrt:.1} |
| transient HOT (50 °C), σ | {sig_hot} MPa | `materials::pla::SIG_ALLOW_HOT` | ×{mhot:.1} |

**Why creep governs.** A shelf bracket holds its load continuously, so the
short-term allowable answers the wrong question (it covers someone leaning
on the shelf — that is the RT row). The sustained number is read live from
`materials::pla::creep_allowable_mpa({t_c}, {t_h})`: 23 °C because this
is an indoor wall bracket, 1 year
because that is the longest cell the table carries. The table's own
confidence note calls that cell a conservative extrapolation (no printed-PLA
dataset beyond ~170 h exists), and its construction already includes a
safety factor of 2.0 on the worst measured printed rupture — so the margin
above sits on top of an already-conservative number.

**Anisotropy.** The side-lying print puts the bending stress IN the layer
plane ({oop:.1}° out of plane, threshold 30°), so the record's across-layer
knockdown (×0.55) does NOT apply. Printed upright it would, and the
sustained margin would fall to ×{mcreep_z:.2} — this is why the shipped pose
is not a preference.

The short-term chain: base 35 MPa × 0.6 layer adhesion × 0.5 design factor.

## Fasteners (closed-form, load path: tip load → wall couple)

Conservative throughout: full-projection lever, the couple reacted by the
screw pair alone, and the seat area discounted by {seatfrac} for the crown
roof's cap.

- top-screw tension = F·proj/spread = {LOAD_N:.1}·{PROJ:.0}/{spread:.0} ≈ **{tens:.0} N**.
  Pull-out: an M4 in any Ø6 plug is rated in the hundreds of N — margin >4×.
- **screw seat bearing is the governing DETAIL of this design.** Sustained,
  so it is checked against the creep allowable, not the short-term one:
  - with the specified Ø{WASHER_OD:.0} countersunk washer: **{seat:.2} MPa**
    vs {sig_creep} MPa → margin ×{seatm:.2}
  - with a bare DIN 7991 M4 head (Ø8): {bare:.2} MPa → **over the line**
  That is why the washer is in the BOM as a required part rather than a
  suggestion. It is also the honest weak point: the printed cone under a
  steel head is where this bracket would creep first, not the web.
- Caveat stated rather than buried: this compares a COMPRESSIVE contact
  pressure against a TENSILE creep allowable, because no compressive-creep
  data for printed PLA exists in the record. That is conservative (PLA is
  stronger in compression, and the contact is confined), but it is an
  inequality between unlike quantities and is labelled as such.
- Plate bearing on the wall and screw shear are lower still — not governing.

## Buckling of the web's compression diagonals

A topology-optimized web is a truss, and its lower diagonal runs in
compression — the one place elastic buckling could beat strength. Receipts:
`fea/buckle_final_receipt.json`.

- linear buckling load factor λ = **{lam:.1}** on the rated reference load
  ({appl:.1} N half-model) → critical {crit:.0} N
- FDM knockdown ×{kd:.2} (the runner's cited default: AISC 0.877 for straight
  steel, EN 1993 imperfection curves, NASA SP-8007 0.32–0.65 for shells —
  printed parts are more imperfect than any of those calibration sets)
  → **design critical {dcrit:.0} N = ×{bmarg:.1} the rated load**
- Linear bifurcation on perfect geometry is an UPPER bound; the knockdown is
  why the gate uses the knocked-down number. Verdict: buckling is **not**
  the governing failure mode — creep is.

## Analysis plan (per DESIGN_GUIDE §25 step 7 — every required item answered)

| analysis | required? | status |
|---|---|---|
| static structural (stiffness + strength) | yes | **receipts** — ACE FEA baseline/final above, gated |
| **creep / sustained load** | **yes — this is a permanently loaded part** | **researched-table receipt** — governing gate above, `materials::pla::creep_allowable_mpa(23, 8760)` = {sig_creep} MPa |
| buckling of web compression members | yes — an optimized web has slender compression diagonals | **receipts** — `tools/ace_buckling_runner.py`, ×{bmarg:.1} knocked-down margin above |
| fastener pull-out / seat bearing | yes | **closed-form above, gated** — seat bearing vs the SUSTAINED allowable; the required washer is what passes it |
| print-readiness (overhang/bridge/bed/feature) | yes | **receipts** — support audit + thin-wall gates this run |
| thermal | **no** | there is no heat source: an indoor wall bracket sits at room ambient. The temperature dependence that DOES matter is captured by choosing the 23 °C creep cell; the 50 °C transient row bounds a hot siting. Running a conduction solve on a part with no thermal load would be plan padding |
| modal / vibration | **no** | static furniture with no excitation source; a wall bracket's first mode is irrelevant to any failure path here |
| fatigue | **no** | shelf load is applied once and held — that is creep, not cycling. Loading/unloading a shelf a few hundred times over its life is nowhere near a fatigue regime |

## Print

- {mo:.0} g PLA solid-equivalent (print solid: the topology IS the infill);
  coupon {mc:.0} g. Side-lying, worst bridge ≤ {br:.1} mm.
- **Support: {steep:.1} mm² flagged against a {budget:.1} mm² budget**
  ({steeppc:.3}% of the surface), and every flagged patch sits within
  {worstr:.1} mm of a screw axis — the WEB itself audits clean. This residual
  is not a design overhang: the exact B-rep coupon carries the identical
  screw features and audits at 0.000 mm². It is the mesher rounding the
  sharp apex ridge of each teardrop roof into a ~half-voxel band, so it
  scales with MESH_VOX ({MESH_VOX} mm), not with the design, and lands as
  ~0.2 mm-wide strips a slicer lays down as ordinary extrusion. Print it
  without supports.
- Thin wall: the probe reads {tw:.2} mm on the optimized body and
  {twc:.2} mm on a SOLID control of the same silhouette — i.e. the floor is
  set by TAPERS (material running out at a boundary), not by the topology,
  and the two are indistinguishable at the probe's {pcell:.2} mm resolution.
  The number that answers printability is the minimum MEMBER width, which
  the SIMP filter radius imposes at 2·{SIMP_FILTER_RVOX}·{VOX} = {lscale:.1} mm
  and which measures 3.50 mm on the rebuilt section (DESIGN.md).
- Negative controls this run: upright-orientation audit fired
  ({nc_steep:.0} mm² steep), mutilated-geometry FEA fired ({nc_ratio:.1}× tip),
  thin-wall oracle fired on a 0.8 mm slab.
"#,
		mb = vol_base * PLA,
		tb = tip_base,
		vb = vm_base,
		mo = vol_opt * PLA,
		tf = tip_final,
		vf_ = vm_final,
		mr = mass_red * 100.0,
		ratio = tip_final / tip_base,
		vr = vm_final / vm_base,
		it = f(&opt_a, &["iterations"]),
		stop = opt_a["stop_reason"].as_str().unwrap_or("?"),
		c0 = f(&opt_a, &["compliance_first"]),
		c1 = f(&opt_a, &["compliance_last"]),
		vfa = vf,
		vm_d = vm_design,
		mrt = sig_rt / vm_design,
		mhot = sig_hot / vm_design,
		mcreep = sig_creep / vm_design,
		mcreep_z = sig_creep * materials::pla::Z_VS_XY_STRENGTH_RATIO / vm_design,
		t_c = CREEP_DESIGN_T_C,
		t_h = CREEP_DESIGN_HOURS,
		oop = out_of_plane_deg,
		lam = lam,
		appl = applied_ref,
		crit = f(&buckle, &["critical_load_N"]),
		kd = f(&buckle, &["knockdown", "recommended_factor"]),
		dcrit = design_crit,
		bmarg = buckle_margin,
		tens = screw_tension,
		spread = SCREW_TOP_Z - SCREW_BOT_Z,
		seat = seat_mpa,
		seatm = sig_creep / seat_mpa,
		bare = bare_head_mpa,
		seatfrac = SEAT_BEARING_FRAC,
		mc = volume(&coupon).abs() * PLA,
		br = 0.0f64.max({
			let m = mesh_posed(&m_opt, print_pose());
			m.support_free_report(Vec3::Z, 45.0, 0.3).max_bridge_span
		}),
		tw = tw.thinnest,
		twc = tw_ctl.thinnest,
		pcell = probe_cell,
		lscale = 2.0 * SIMP_FILTER_RVOX * VOX,
		steep = rep_final.steep_area,
		budget = SUPPORT_BUDGET_FRAC * rep_final.total_area,
		steeppc = 100.0 * rep_final.steep_area / rep_final.total_area,
		worstr = worst_r,
		nc_steep = rep_wrong.steep_area,
		nc_ratio = tip_nc / tip_final,
	);
	let _ = std::fs::write(format!("{FAM}/analysis/ANALYSIS.md"), analysis);

	// authored DESIGN.md (loop narrative + route decisions), README, listing,
	// and the regeneration script — all written by the example so the family
	// is reproducible from `cargo run` alone.
	let _ = std::fs::write(format!("{FAM}/analysis/DESIGN.md"), design_md());
	let _ = std::fs::write(format!("{FAM}/README.md"), readme_md(vol_opt * PLA, mass_red * 100.0, tip_final / tip_base));
	let _ = std::fs::write(format!("{FAM}/publish/PRINTABLES_LISTING.md"), listing_md(vol_opt * PLA, vol_base * PLA, mass_red * 100.0, tip_final / tip_base));
	let _ = std::fs::write(format!("{FEA_DIR}/run_opt.sh"), run_opt_sh());

	println!(
		"\nloop: {:.0} g -> {:.0} g (−{:.1}%), tip {:.4} -> {:.4} mm ({:.2}×), peak {:.2} MPa (RT margin ×{:.1})",
		vol_base * PLA,
		vol_opt * PLA,
		mass_red * 100.0,
		tip_base,
		tip_final,
		tip_final / tip_base,
		vm_final,
		sig_rt / (vm_final * HEX8_PEAK_FACTOR),
	);
	println!("\nBRACKET GEN: {}", if ok { "ALL GATES PASS" } else { "<<< FAIL" });
	std::process::exit(if ok { 0 } else { 1 });
}

/// Run a python tool whose receipt is plain stdout (render_views.py), not JSON.
fn run_py_plain(tool: &str, args: &[&str]) -> Result<(), String> {
	let out = std::process::Command::new("python3")
		.arg(tool)
		.args(args)
		.output()
		.map_err(|e| format!("python3 not runnable: {e}"))?;
	if out.status.success() {
		Ok(())
	} else {
		Err(format!("{tool} exited {:?}: {}", out.status.code(), String::from_utf8_lossy(&out.stderr).chars().take(200).collect::<String>()))
	}
}

fn design_md() -> String {
	format!(
		r#"# GEN BRACKET — design contract (the generative loop, narrated)

A 10 kg wall/shelf bracket, {PROJ:.0}×{HEIGHT:.0}×{WIDTH:.0}, whose web is
designed by SIMP topology optimization and whose every claim is re-proved by
`crates/kernel-model/examples/bracket_gen.rs` on every run (exit 1 on FAIL).
Numbers live in ANALYSIS.md (generated); this file records the decisions.

## The loop (each stage leaves receipts in analysis/fea/)

1. **Baseline** — solid right-triangle web, exact B-rep, M4 stations cut by
   the hole wizard (DIN 74 countersink + ISO 273 bore). `fea_baseline_receipt`
   is its measured mass/stiffness reference.
2. **Optimize** — `tools/ace_optimize_runner.py` (SIMP + density filter + OC,
   top88 lineage, driven by ACE's hex8 FEA in SIMP mode) at volfrac {VOLFRAC}
   on the design region. Keep-outs are the real geometry: FROZEN = wall plate
   (screw bearing + clamped face) and shelf strip (the load pad's seat);
   VOID = the two driver tunnels, so the optimizer routes material around the
   holes the rebuild bores rather than counting their cores as structure.
   Run TWICE; `final_rho.npy` must come back byte-identical (determinism).
3. **Geometry from density** — `final_rho.npy` → `GridField` → **threshold +
   smooth**: y-averaged to a 2.5D extruded web, one 3×3 tent blur, iso {ISO},
   plus a continuous {CHORD_T} mm outer chord just inside the hypotenuse.
   *Why not graded infill:* the product claim is a topology-optimized SHAPE;
   and a 2.5D web lying on its side is what makes the print support-free —
   an organic 3D lattice would not be. The y-averaging is a stated
   manufacturing regularization; whatever it costs shows up honestly in the
   final re-analysis, which is the gate.
   *Why the chord:* SIMP wants material along the whole diagonal but leaves
   it hovering AT the iso threshold, so the iso-surface ran tangent to the
   hypotenuse clip and tapered members into knife edges. A truss needs a
   chord. It is ONE-SIDED (inside the silhouette): a symmetric band has half
   its width outside, and clipping that produced fresh slivers (measured:
   sub-1.6 mm samples 4 → 8, and only the one-sided 4 mm strip reaches 0).
4. **Back to exact** — the v1 faceted bridge is built first, then
   `implicit_to_solid_recovered` runs as a finishing pass and is **kept only
   if it passes the same validity + watertight gates the shipped part is
   held to**. On this geometry it does not: recovery fits ~6 cylinder
   carriers at ~0.076 mm residual and the rebuilt solid tessellates
   non-watertight (the sector-merge/boundary-ring limit stated in the
   `reverse` module doc). That is not cosmetic — shipping it fed a leaky STL
   to the voxelizer, whose parity fill then reported a 143× tip deflection.
   Which route ran is printed and recorded in ANALYSIS.md every build.
   Screw stations are re-cut on the implicit body with constants PINNED to
   the wizard's M4 table by a gate; the wizard itself cuts the exact baseline
   and the shipped coupon, where exact-boolean operands exist.
5. **Honest re-analysis** — the FINAL rebuilt geometry is re-voxelized and
   re-solved through the SAME manifest (fixtures/load byte-identical). Gates:
   mass −{MASS_RED_MIN_PC:.0}% floor, tip ≤ {STIFF_FACTOR}× baseline, and
   the stress rows — of which the GOVERNING one is sustained-load creep
   (23 °C / 1 y from `materials::pla::creep_allowable_mpa`), not the
   short-term allowable: a shelf bracket is loaded permanently. Buckling of
   the web's compression diagonals gets its own solve. The SIMP-internal
   homogenized numbers are never quoted as part properties.
6. **Negative controls** — wrong print orientation must trip the support
   audit; a mutilated bracket through the same FEA chain must deflect ≥1.5×;
   the thin-wall probe must fire on a 0.8 mm slab. A gate that cannot fail
   is not a gate.

## How the parameters were chosen (measured, not guessed)

The first build of this campaign FAILED five gates, and each fix is a
measurement rather than a loosened threshold:

| symptom | cause | fix |
|---|---|---|
| tip deflection 143× baseline, NC indistinguishable | v2-recovered solid tessellated non-watertight → `voxelize_stl.py` parity-filled a leaky mesh into garbage occupancy | route selection now GATES the recovery pass (step 4) |
| thinnest feature 0.03 mm; 2–3 disconnected pieces | SIMP filter radius 1.5 vox left necks between diagonals | filter radius → {SIMP_FILTER_RVOX} vox (imposed length scale ≫ nozzle); measured thinnest 0.50 → 3.50 mm, pieces 3 → 1 |
| members tapering to knife edges at the hypotenuse | iso-surface tangent to the silhouette clip | one-sided {CHORD_T} mm chord (step 3) |
| stiffness vs mass trade unpinned | volfrac picked by guess | swept 0.42 / 0.34 / 0.30 against the {STIFF_FACTOR}× gate → {VOLFRAC} |

## Modeling honesty (the fine print that matters)

- **Half model + symmetry slider**: geometry, load and fixtures are
  y-symmetric; the slider fixes u_y on the y≈0 element layer (both node
  planes of that layer — marginally over-stiff in u_y, negligible for a
  symmetric problem).
- **Clamped wall face**: the whole back-face voxel layer is clamped — the
  standard bracket-TO idealization; real compliance lives in the wall/anchor,
  which is why fastener forces are also checked closed-form.
- **Frozen regions re-pin drilled voxels to 1.0** (ACE region precedence:
  frozen wins over the baseline's holes inside the plate/boss bands). The
  SIMP model is therefore locally slightly stiffer than reality there; the
  final binary FEA of the actually-drilled part is what gates the claim.
- **Pseudo-SDF, not a distance field**: (iso − ρ)·{SDF_SCALE} is a sign-correct
  bound; meshing is sample-based dual contouring, and the thin-wall probe's
  2·|d| reading is a lower bound (§12.3 honesty).
- **Teardrop everything horizontal**: driver tunnels Ø{TUNNEL_D:.0}, overbore
  Ø{BORE_D}, and a 47° roof prism over the countersink crown (the 90° cone's
  crown facets sit at 45.0° — exactly the audit limit — so they are roofed
  away rather than argued about).

## Print & use

Side-lying (the shipped pose), 0.2 mm layers, 3–4 perimeters, print SOLID —
the optimized topology replaces infill. Two brackets per shelf; M4×30
DIN 7991 into Ø6 plugs. The coupon in optional/ proves the countersink seat
and the teardrop roofs on YOUR printer in 15 minutes.
"#,
		MASS_RED_MIN_PC = MASS_RED_MIN * 100.0,
	)
}

fn readme_md(mass_g: f64, red_pc: f64, ratio: f64) -> String {
	format!(
		r#"# GEN BRACKET — a topology-optimized 10 kg shelf bracket

A {PROJ:.0} mm × {HEIGHT:.0} mm PLA wall bracket whose web was designed by
SIMP topology optimization and re-verified as the printed binary geometry:
{mass_g:.0} g (−{red_pc:.0}% vs the solid baseline) at {ratio:.2}× the
baseline tip deflection, carrying 10 kg at the shelf tip on ×2-derated PLA
allowables. Prints on its side with zero supports; mounts with two M4
countersunk screws.

## Folder map

| you're asking… | open |
|---|---|
| what do I print? | `parts/` (the bracket) · `optional/` (M4 seat coupon — print first) |
| how does it mount? | `assembly/` — ASSEMBLY.png, BOM.md, instructions.md |
| can I modify it? | `cad/` — baseline + optimized STEP |
| what does it look like? | `renders/` |
| is it verified? | `analysis/` — ANALYSIS.md (generated), DESIGN.md, fea/ receipts |
| how do I share it? | `publish/` |

## Print

| file | qty | notes |
|---|---|---|
| `optional/coupon_m4_seat` | 1 first | 15 min: M4 flat-head must pull flush; teardrop roofs must print clean |
| `parts/bracket_optimized` | 2 per shelf | as-posed (side-lying), 0.2 mm layers, 3–4 walls, **solid** (the topology is the infill), no supports |

## Mount (per bracket)

1. Level, mark through both bores, drill Ø6, fit plugs (or hit a stud).
2. Drop an **M4 Ø10 countersunk (cup) washer** into each countersink — it is
   required, not optional: it is what keeps the seat under PLA's sustained
   creep allowable. Then drive two M4×30 DIN 7991 through the teardrop
   tunnels until the heads sit flush.
3. Set the shelf on the arm. Rated 10 kg per bracket at the tip.

## What is machine-verified on every build (exit-gated)

The ENTIRE generative loop: baseline FEA receipts → SIMP (twice —
byte-identical determinism gate) → density→implicit→exact rebuild → honest
FEA re-analysis of the final binary geometry (mass, tip stiffness, stress vs
PLA allowables, creep-governed) → buckling → support/bridge/bed/thin-wall
print audit (the flagged area is a measured, budgeted mesher artifact at the
screw roofs — the web audits clean; see ANALYSIS.md) → three
negative controls (wrong orientation, mutilated geometry, thin-wall slab).
Numbers: `analysis/ANALYSIS.md`. Method: `analysis/DESIGN.md`.
"#
	)
}

fn listing_md(mass_g: f64, base_g: f64, red_pc: f64, ratio: f64) -> String {
	format!(
		r#"# Printables listing — copy-paste content

Lead the gallery with `assembly/ASSEMBLY.png`, then `renders/render_bracket.png`
("render") and a photo of the printed bracket under load.

---

## Name

GEN BRACKET — Topology-Optimized 10 kg Shelf Bracket (M4, support-free)

## Summary

A 100×150 mm PLA shelf bracket designed by real topology optimization and
re-verified as printed geometry: {mass_g:.0} g instead of the {base_g:.0} g
solid baseline (−{red_pc:.0}%) at just {ratio:.2}× the tip deflection. Rated
10 kg per bracket on derated printed-PLA allowables. Prints on its side with
zero supports; every screw hole is teardropped.

## Description

**What it is** — one printed bracket + two M4×30 countersunk screws + two
M4 Ø10 cup washers (required — see below) + two Ø6 wall plugs. The web shape came out of a SIMP topology-optimization loop
(ACE hex8 FEA), was rebuilt as an exact CAD solid, and — the part that
matters — the FINAL printed geometry was re-analyzed: {ratio:.2}× baseline
tip deflection at −{red_pc:.0}% mass, peak stress under the room-temperature
design allowable for printed PLA with the coarse-mesh caveat derated in.

**Print** — side-lying as shipped, 0.2 mm layers, 3–4 walls, SOLID (the
topology replaces infill), no supports anywhere: the driver tunnels and the
M4 bores are teardropped, and the countersink crowns carry a 47° roof.

**Verify before the long print** — `coupon_m4_seat` is a 15-minute block
with the exact screw station: the flat head must pull flush.

**The washers are not optional.** The bracket is loaded permanently, so it
is designed against PLA's *sustained* (creep) allowable rather than its
short-term strength. A bare M4 head bears on too little printed cone to stay
under that line; a 2-cent cup washer fixes it with margin. Every other
number in the analysis has more room than this one.

**Load rating** — 10 kg at the shelf tip per bracket (use two per shelf).
The rating sits on top of PLA allowables that already carry a 0.6 layer
adhesion × 0.5 design factor chain — this is an engineering rating, not a
survived-once number.

## Print settings

PLA · 0.2 mm · 3–4 perimeters · 100% infill · no supports · side-lying as shipped

## Files

parts/bracket_to.stl (+3mf), optional/coupon_m4_seat.stl, cad/*.step
(baseline + optimized), full FEA/SIMP receipts in analysis/fea/.
"#
	)
}

fn run_opt_sh() -> String {
	r#"#!/bin/sh
# Regenerate every receipt of the GEN BRACKET generative loop from the saved
# manifests. The occupancy grids (*_occ.npy) each job reads are SAMPLED FROM
# THE FIELD by the campaign example, so run that first — or just run it
# instead: it does all of this and gates the results.
#   cargo run --release -p kernel-model --example bracket_gen
cd "$(dirname "$0")/../../../.." || exit 1
set -x
python3 tools/ace_fea_runner.py  bracket_system/gen_bracket/analysis/fea/fea_baseline.json
python3 tools/ace_optimize_runner.py bracket_system/gen_bracket/analysis/fea/opt_a.json
python3 tools/ace_optimize_runner.py bracket_system/gen_bracket/analysis/fea/opt_b.json
python3 tools/ace_fea_runner.py  bracket_system/gen_bracket/analysis/fea/fea_final.json
python3 tools/ace_fea_runner.py  bracket_system/gen_bracket/analysis/fea/fea_nc.json
python3 tools/ace_buckling_runner.py bracket_system/gen_bracket/analysis/fea/buckle_final.json
"#
	.to_string()
}
