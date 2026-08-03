// Copyright (c) LMCAD. Licensed under the MIT License.

//! Periodic strut/beam lattices — the strut-based counterpart of the six TPMS
//! families ([`Tpms`](crate::primitives::Tpms)).
//!
//! A [`StrutLattice`] is an UNBOUNDED triply-periodic field: a unit cell's strut
//! segments (each swept by a uniform `radius`) tiled with period `cell` over all
//! of space. `distance(p)` = (min over the periodic images of the unit-cell
//! segments of the point→segment distance) − `radius`. Like a TPMS you bound it
//! by intersecting with a shroud solid; unlike the trig TPMS fields the value
//! here is a true Euclidean distance. It complements the finite
//! [`BeamLattice`](crate::lattice::BeamLattice) (an explicit node/strut graph in
//! a box with a grid-accelerated query): use `BeamLattice` for large one-off
//! graphs, `StrutLattice` for the periodic lattice vocabulary (BCC / FCC /
//! octet, or a custom unit cell via [`graph_lattice`]).
//!
//! # Lipschitz / exactness contract (load-bearing for narrow-band meshing)
//!
//! Each segment distance is exact and 1-Lipschitz; a min of 1-Lipschitz fields
//! is 1-Lipschitz, so the lattice field is **exactly 1-Lipschitz** — pinned by
//! `tests/strut.rs` with the secant probe [`probe_lipschitz`]. Outside the
//! union of struts the field equals the exact signed distance; inside an
//! overlap of several struts the `min` can only *understate* the depth, never
//! overstate it — safe for [`crate::narrow_band`] block pruning. Because of
//! that inside-overlap understatement it is a distance BOUND, not an exact SDF:
//! wrap it with [`Node::primitive_bound`](crate::ops::Node::primitive_bound)
//! (not `primitive`), exactly like [`Tpms`](crate::primitives::Tpms), so a
//! downstream `offset`/`shell` is honestly flagged approximate.
//!
//! # Tiling correctness (the classic seam bug, solved by proof)
//!
//! A query is folded into the base cell (`q = p mod cell`, componentwise into
//! `[0, cell)`); the folded point is then evaluated against a **pre-baked list
//! of segment images**: every base segment translated by each `t ∈ {−1, 0, 1}³`
//! cells. This 3³ neighborhood is *provably* enough: for any base segment with
//! endpoints in the closed unit cell and any `q` in it, wrapping one endpoint
//! per axis (each wrap ≤ half a cell) yields an image within `(√3/2)·cell` of
//! `q`, while every image outside the neighborhood is ≥ `1·cell` away — so the
//! 27-image min equals the min over ALL integer translates, i.e. the folded
//! evaluation IS the periodic field, continuous across every cell border.
//!
//! Two tempting shortcuts are wrong, and this module deliberately avoids them:
//! - *"Only the base segments"*: a strut reaching (or bulging within `radius`
//!   of) a border would vanish when seen from the neighboring cell — the seam
//!   crack that shows up as notched rods and open mesh edges.
//! - *"Only the images that reach into the cell"*: the nearest image need not
//!   touch the cell at all (a mid-cell segment's `−x` image can sit `0.4·cell`
//!   from a query near the `x = 0` face while the in-cell copy is `0.5·cell`
//!   away), so that pruning would still jump across the border. The correct
//!   prune, used here, keeps an image iff its AABB is within the `(√3/2)·cell`
//!   per-segment covering radius of the closed cell — images beyond it can
//!   never win against the segment's own best wrap, so dropping them provably
//!   never changes the field. Bit-identical duplicate images (shared faces /
//!   edges of the tiling) are removed once at construction.
//!
//! Cost is `O(images)` per query — a few hundred segment distances for the
//! standard kinds ([`StrutLattice::image_count`]) — which meshes comfortably;
//! junction-rich lattices should be extracted with
//! [`manifold_dual_contour`](crate::manifold_dc::manifold_dual_contour) (the
//! one-vertex-per-cell duals fold multi-strut junction saddles into
//! non-manifold fins; measured in `lattice.rs`).

use std::collections::HashSet;

use kernel_core::math::{Aabb, DVec3, Vec3};
use kernel_core::sdf::Sdf;

use crate::lattice::Pipe;

/// Squared per-segment wrap covering radius `(√3/2)²`, in cell units. For any
/// base segment (endpoints in the closed unit cell) and any query in the cell,
/// the per-axis wrap of an endpoint is ≤ 0.5 cells, so the segment's own best
/// image is within `√3/2` cells — an image whose AABB is farther than that from
/// the cell can never be that segment's argmin (see the module docs).
const COVER_RADIUS_SQ: f32 = 0.75;

/// Unit-cell topology families for [`StrutLattice`] (strut coordinates in
/// `[0, 1]³` cell space; every strut is repeated with period `cell`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StrutKind {
	/// Body-centred cubic: the 8 corner↔body-center struts (an X through the
	/// cell; length `cell·√3/2`). Bending-dominated → compliant/springy.
	Bcc,
	/// Face-centred cubic: the 24 corner↔face-center struts (each of the 6 face
	/// centers to its 4 face corners; all lie in the cell faces, so tiled they
	/// are shared between neighbors; length `cell/√2`). Equivalently, every
	/// corner links to its 12 nearest face centers.
	Fcc,
	/// The standard octet truss (Deshpande et al.): the FCC vertex set (corners
	/// plus face centers) with the 24 corner↔face struts AND the 12 face↔face
	/// diagonals (the inner octahedron connecting non-opposite face centers).
	/// All 36 struts have length `cell/√2`. Stretch-dominated — the structural
	/// workhorse.
	Octet,
}

impl StrutKind {
	/// The unit-cell strut set in `[0, 1]³` cell coordinates (exact halves, so
	/// tiled duplicates dedup bit-exactly at bake time).
	fn base_segments(self) -> Vec<(Vec3, Vec3)> {
		let corner = |i: u32| Vec3::new((i & 1) as f32, ((i >> 1) & 1) as f32, ((i >> 2) & 1) as f32);
		// Face centers ordered [x−, x+, y−, y+, z−, z+] (mirrors lattice.rs octet).
		let face = |f: usize| {
			let mut c = [0.5f32; 3];
			c[f / 2] = (f % 2) as f32;
			Vec3::from_array(c)
		};
		let corner_face = || -> Vec<(Vec3, Vec3)> {
			let mut out = Vec::with_capacity(24);
			for f in 0..6usize {
				let fc = face(f);
				let (a1, a2) = match f / 2 {
					0 => (1, 2),
					1 => (0, 2),
					_ => (0, 1),
				};
				for s in 0..4u32 {
					let mut c = fc.to_array();
					c[a1] = (s & 1) as f32;
					c[a2] = ((s >> 1) & 1) as f32;
					out.push((fc, Vec3::from_array(c)));
				}
			}
			out
		};
		match self {
			StrutKind::Bcc => (0..8).map(|i| (corner(i), Vec3::splat(0.5))).collect(),
			StrutKind::Fcc => corner_face(),
			StrutKind::Octet => {
				let mut out = corner_face();
				for a in 0..6usize {
					for b in (a + 1)..6 {
						if a / 2 != b / 2 {
							out.push((face(a), face(b)));
						}
					}
				}
				out
			}
		}
	}
}

/// A triply-periodic strut lattice as an [`Sdf`]: a unit-cell strut set swept
/// by `radius` and tiled with period `cell` (see the module docs for the
/// tiling proof and the Lipschitz/exactness contract).
///
/// Construct via [`StrutLattice::new`] (a [`StrutKind`] family) or
/// [`graph_lattice`] (a custom unit cell). The field is periodic everywhere;
/// `region` is only the [`bounds`](Sdf::bounds) hint (mirroring
/// [`Tpms`](crate::primitives::Tpms)) — intersect with a shroud solid to bound
/// the lattice, and wrap with
/// [`Node::primitive_bound`](crate::ops::Node::primitive_bound).
#[derive(Clone)]
pub struct StrutLattice {
	/// Bounds hint returned by [`Sdf::bounds`]; safe to reassign (it never
	/// affects the field). [`graph_lattice`] sets the infinite box, like
	/// [`Plane`](crate::primitives::Plane) — a bare unbounded lattice is
	/// rejected by the meshers' finite-domain guard until shrouded.
	pub region: Aabb,
	cell: f32,
	radius: f32,
	/// Pre-baked segment images in world units, covering `{−1, 0, 1}³` cell
	/// translates of the base set (deduplicated, covering-radius pruned).
	images: Vec<(Vec3, Vec3)>,
}

impl StrutLattice {
	/// Periodic lattice of `kind` with unit-cell period `cell` and uniform
	/// strut `radius`, reporting `region` as its meshing bounds.
	///
	/// Contract (asserted): `cell` and `radius` positive and finite. A `radius`
	/// at or above the cell's covering radius simply yields a fully solid field
	/// — legal, just no longer a lattice.
	pub fn new(region: Aabb, kind: StrutKind, cell: f32, radius: f32) -> Self {
		Self::build(region, &kind.base_segments(), cell, radius, "StrutLattice")
	}

	/// Shared checked constructor for [`StrutLattice::new`] / [`graph_lattice`].
	fn build(region: Aabb, edges: &[(Vec3, Vec3)], cell: f32, radius: f32, who: &str) -> Self {
		assert!(cell > 0.0 && cell.is_finite(), "{who}: cell must be positive and finite, got {cell}");
		assert!(radius > 0.0 && radius.is_finite(), "{who}: radius must be positive and finite, got {radius}");
		assert!(!edges.is_empty(), "{who}: the unit-cell strut set must not be empty");
		for (i, &(a, b)) in edges.iter().enumerate() {
			let inside = |v: Vec3| v.is_finite() && v.cmpge(Vec3::ZERO).all() && v.cmple(Vec3::ONE).all();
			assert!(
				inside(a) && inside(b),
				"{who}: edge {i} ({a:?} → {b:?}) must have both endpoints in [0, 1]³ cell space — the 3³-translate tiling is only exact for segments inside the closed unit cell"
			);
		}
		Self { region, cell, radius, images: bake_images(edges, cell) }
	}

	/// Unit-cell period in world units.
	pub fn cell(&self) -> f32 {
		self.cell
	}

	/// Uniform strut radius in world units.
	pub fn radius(&self) -> f32 {
		self.radius
	}

	/// Number of pre-baked segment images evaluated per query (after dedup and
	/// covering-radius pruning) — the per-query cost, for sizing decisions.
	pub fn image_count(&self) -> usize {
		self.images.len()
	}
}

impl Sdf for StrutLattice {
	fn distance(&self, p: Vec3) -> f32 {
		// Fold into the base cell: q ∈ [0, cell)³. Periodicity makes this exact;
		// far from the origin the fold inherits f32 quantization of `p` itself
		// (the same caveat as the trig TPMS fields).
		let q = p - (p / self.cell).floor() * self.cell;
		let mut best = f32::INFINITY;
		for &(a, b) in &self.images {
			best = best.min(seg_distance(q, a, b));
		}
		best - self.radius
	}

	fn distance64(&self, p: DVec3) -> f64 {
		let cell = self.cell as f64;
		let q = p - (p / cell).floor() * cell;
		let mut best = f64::INFINITY;
		for &(a, b) in &self.images {
			best = best.min(seg_distance64(q, a.as_dvec3(), b.as_dvec3()));
		}
		best - self.radius as f64
	}

	fn bounds(&self) -> Aabb {
		self.region
	}
}

/// Periodic lattice from a **caller-supplied unit-cell strut set**: `edges` are
/// segments in `[0, 1]³` cell space (asserted — the tiling proof needs the
/// closed unit cell), swept by `radius` and tiled with period `cell`. Same
/// machinery, tiling guarantee and 1-Lipschitz contract as the built-in
/// [`StrutKind`] families (module docs). Segments touching opposite faces
/// continue seamlessly into the neighboring cells (e.g. one edge
/// `(0, ½, ½) → (1, ½, ½)` yields infinite continuous rods along x).
///
/// The returned lattice reports the INFINITE box as bounds (the field is
/// genuinely unbounded, mirroring [`Plane`](crate::primitives::Plane)):
/// intersect with a shroud solid — which supplies the finite bound — or assign
/// [`StrutLattice::region`] yourself. Duplicate and zero-length edges are
/// legal (a degenerate segment is its end sphere). Intended for unit cells of
/// up to a few hundred struts; for large one-off graphs use the
/// grid-accelerated [`BeamLattice`](crate::lattice::BeamLattice).
pub fn graph_lattice(edges: &[(Vec3, Vec3)], cell: f32, radius: f32) -> StrutLattice {
	let infinite = Aabb::new(Vec3::splat(f32::NEG_INFINITY), Vec3::splat(f32::INFINITY));
	StrutLattice::build(infinite, edges, cell, radius, "graph_lattice")
}

/// NON-tiled capsule chain along a polyline — the skeleton/pipe modeling
/// primitive: every consecutive point pair becomes a capsule of uniform
/// `radius`, evaluated as the exact min-union (so the field is **exactly
/// 1-Lipschitz**, exact outside the union, understating only inside overlaps —
/// the [`Pipe`](crate::lattice::Pipe) contract, which also provides the
/// grid-accelerated query and `volume_estimate`). Per-vertex radii or a helix
/// want [`Pipe`](crate::lattice::Pipe) directly; this is the uniform-radius
/// convenience the strut vocabulary pairs with.
///
/// Contract (asserted, via `Pipe::new`): ≥ 2 finite points, positive finite
/// `radius`.
pub fn pipe_path(points: &[Vec3], radius: f32) -> Pipe {
	Pipe::new(points.to_vec(), vec![radius; points.len()])
}

/// Directions probed by [`probe_lipschitz`]: the 3 axes, 6 face diagonals and
/// 4 body diagonals (normalized at use). Axis-only probing would under-observe
/// fields whose surface normals avoid the axes (a BCC strut wall's normal is
/// everywhere ≥ 35° from every axis).
const PROBE_DIRS: [Vec3; 13] = [
	Vec3::new(1.0, 0.0, 0.0),
	Vec3::new(0.0, 1.0, 0.0),
	Vec3::new(0.0, 0.0, 1.0),
	Vec3::new(1.0, 1.0, 0.0),
	Vec3::new(1.0, -1.0, 0.0),
	Vec3::new(1.0, 0.0, 1.0),
	Vec3::new(1.0, 0.0, -1.0),
	Vec3::new(0.0, 1.0, 1.0),
	Vec3::new(0.0, 1.0, -1.0),
	Vec3::new(1.0, 1.0, 1.0),
	Vec3::new(1.0, 1.0, -1.0),
	Vec3::new(1.0, -1.0, 1.0),
	Vec3::new(-1.0, 1.0, 1.0),
];

/// Sample-verify a field's Lipschitz constant: the maximum observed **secant
/// slope** `|d(a) − d(b)| / |a − b|` over an `n³` lattice spanning `domain`,
/// probing short steps (`diagonal/4096`) along [`PROBE_DIRS`] plus
/// lattice-neighbor pairs along the axes. Returns `+∞` if any sample is
/// non-finite (a broken field is never certified).
///
/// This is the [`Sdf`]-level sibling of `kernel-api`'s `probe_lipschitz`
/// (which checks the DECLARED `lipschitz_bound` of JSON `expr_sdf` leaves
/// before a narrow-band mesh); use it to gate that a field claiming
/// `≤ 1`-Lipschitz (TPMS, strut lattices, pipes) really is, e.g.
/// `probe_lipschitz(&lat, domain, 16) <= 1.0 + 1e-2`.
///
/// Secants, not gradients — deliberately: min-union fields have creases where
/// a composite forward-difference gradient overshoots to `√3` even though the
/// field is truly 1-Lipschitz (each one-sided axis difference picks a
/// different branch), so a gradient-norm probe would raise false alarms. A
/// secant NEVER exceeds the true Lipschitz constant beyond floating-point
/// rounding, for any field. Honest caveat (same as the kernel-api probe): a
/// sampled maximum is a lower bound on the true constant, not a certificate —
/// it reliably catches gross violations, not a peak between samples.
pub fn probe_lipschitz<S: Sdf + ?Sized>(sdf: &S, domain: Aabb, n: usize) -> f32 {
	assert!(n >= 2, "probe_lipschitz: need n >= 2 lattice points per axis, got {n}");
	assert!(
		domain.is_valid() && domain.min.is_finite() && domain.max.is_finite(),
		"probe_lipschitz: domain must be a finite valid box, got {domain:?}"
	);
	let h = (domain.diagonal() / 4096.0).max(1e-6);
	let step = domain.size() / (n - 1) as f32;
	let at = |i: usize, j: usize, k: usize| domain.min + Vec3::new(i as f32, j as f32, k as f32) * step;
	let idx = |i: usize, j: usize, k: usize| i + n * (j + n * k);
	let mut vals = vec![0.0f32; n * n * n];
	let mut worst = 0.0f32;
	for k in 0..n {
		for j in 0..n {
			for i in 0..n {
				let p = at(i, j, k);
				let v = sdf.distance(p);
				if !v.is_finite() {
					return f32::INFINITY;
				}
				vals[idx(i, j, k)] = v;
				for dir in PROBE_DIRS {
					let w = sdf.distance(p + dir.normalize() * h);
					if !w.is_finite() {
						return f32::INFINITY;
					}
					worst = worst.max((w - v).abs() / h);
				}
			}
		}
	}
	// Mid-scale sanity: secants between adjacent lattice points (skipping
	// degenerate flat axes of the domain).
	for k in 0..n {
		for j in 0..n {
			for i in 0..n {
				let v = vals[idx(i, j, k)];
				if i + 1 < n && step.x > 0.0 {
					worst = worst.max((vals[idx(i + 1, j, k)] - v).abs() / step.x);
				}
				if j + 1 < n && step.y > 0.0 {
					worst = worst.max((vals[idx(i, j + 1, k)] - v).abs() / step.y);
				}
				if k + 1 < n && step.z > 0.0 {
					worst = worst.max((vals[idx(i, j, k + 1)] - v).abs() / step.z);
				}
			}
		}
	}
	worst
}

/// Pre-bake the `{−1, 0, 1}³` cell-translate images of the base segments
/// (cell-unit coordinates in, world-unit segments out), deduplicating
/// bit-identical images (tiling shares faces/edges/corners) and pruning
/// translates provably beyond the per-segment covering radius — see
/// [`COVER_RADIUS_SQ`] and the module docs for why this preserves the exact
/// periodic min while "images that reach into the cell" would not.
fn bake_images(base: &[(Vec3, Vec3)], cell: f32) -> Vec<(Vec3, Vec3)> {
	let unit = Aabb::new(Vec3::ZERO, Vec3::ONE);
	let key = |v: Vec3| [v.x.to_bits(), v.y.to_bits(), v.z.to_bits()];
	// Insert-only set: iteration order is never observed, so the image list
	// order is the deterministic loop order (the R5 lesson).
	let mut seen: HashSet<([u32; 3], [u32; 3])> = HashSet::new();
	let mut images = Vec::new();
	for &(a, b) in base {
		for tz in -1i32..=1 {
			for ty in -1i32..=1 {
				for tx in -1i32..=1 {
					let t = Vec3::new(tx as f32, ty as f32, tz as f32);
					let (ia, ib) = (a + t, b + t);
					if unit.distance_squared_box(Aabb::from_points(&[ia, ib])) > COVER_RADIUS_SQ + 1e-3 {
						continue;
					}
					let (ka, kb) = (key(ia), key(ib));
					let id = if ka <= kb { (ka, kb) } else { (kb, ka) };
					if seen.insert(id) {
						images.push((ia * cell, ib * cell));
					}
				}
			}
		}
	}
	images
}

/// Exact Euclidean distance from `p` to segment `a→b` (the capsule core; a
/// zero-length segment is its point).
#[inline]
fn seg_distance(p: Vec3, a: Vec3, b: Vec3) -> f32 {
	let pa = p - a;
	let ba = b - a;
	let l2 = ba.length_squared();
	if l2 < 1e-12 {
		return pa.length();
	}
	let h = (pa.dot(ba) / l2).clamp(0.0, 1.0);
	(pa - ba * h).length()
}

/// `f64` mirror of [`seg_distance`] (same branch structure).
#[inline]
fn seg_distance64(p: DVec3, a: DVec3, b: DVec3) -> f64 {
	let pa = p - a;
	let ba = b - a;
	let l2 = ba.length_squared();
	if l2 < 1e-12 {
		return pa.length();
	}
	let h = (pa.dot(ba) / l2).clamp(0.0, 1.0);
	(pa - ba * h).length()
}
