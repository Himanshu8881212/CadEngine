// Copyright (c) LMCAD. Licensed under the MIT License.

//! Sparse sampled-SDF caches — [`SparseGrid`] (two-level block-sparse tiles)
//! and [`OctreeGrid`] (adaptive depth): memory **and** build evaluations
//! proportional to the surface band, not the domain volume.
//!
//! Dense caching of a 200 mm domain at 0.2 mm voxels is 1001³ ≈ 10⁹ f32
//! samples (≈ 4 GB) — the ledgered blocker for big domains at fine voxels.
//! The dense-evaluation [`crate::grid::SparseGrid`] already stores sparsely
//! but still *evaluates* every lattice sample at build time; the structures
//! here prune evaluation as well, so build cost also scales with the band.
//!
//! # The Lipschitz-safe allocation proof (shared by both types)
//!
//! The input field `d` must never over-claim distance: `|d(p)| ≤` the true
//! Euclidean distance from `p` to the zero set, and be 1-Lipschitz
//! (`|d(p) − d(q)| ≤ |p − q|`). Every exact SDF qualifies, and so does every
//! honest [`crate::FieldQuality::DistanceBound`] field (CSG `min`/`max`
//! trees, smooth blends, arrays…) — under-claiming only *enlarges* the
//! allocated set, it can never cause a miss. For a tile with centre `c` and
//! covering half-diagonal `h` (no tile point is farther than `h` from `c`),
//! if `|d(c)| > band + h` then for every tile point `p`
//!
//! ```text
//! |d(p)| ≥ |d(c)| − |p − c| ≥ |d(c)| − h > band ≥ 0,
//! ```
//!
//! so the tile contains no zero crossing, no point of the ±`band` shell, and
//! (by continuity along any in-tile path, since `|d| > 0` throughout) a
//! single sign. Contrapositive: every tile touching the surface or its
//! ±`band` shell has `|d(c)| ≤ band + h` and is examined — the centre test
//! **provably cannot miss a surface-crossing tile**. This is the same
//! argument as the Lipschitz-safe coarse scan seeding the narrow-band mesher
//! ([`crate::narrow_band`], `find_seeds`), applied to storage instead of
//! meshing; [`OctreeGrid`] applies it recursively per cell exactly like that
//! scan's octree descent.
//!
//! **Out of contract:** fields that OVER-claim (`|field|` > true distance) —
//! a `k×`-scaled distance, a raw TPMS phase field. For those the pruning is
//! unsound and surface-crossing tiles can be skipped; `tests/sparse.rs` pins
//! a negative control demonstrating the miss for a ×3-scaled sphere. Restore
//! the metric first with [`crate::redistance()`], exactly as
//! [`crate::narrow_band`] prescribes for non-metric fields.
//!
//! # `SparseGrid` field contract (honest labelling)
//!
//! With `vs` = voxel size; a tile owns the 8³ lattice samples of its *stride
//! cube* (the 8³-cell world region `[8t·vs, 8(t+1)·vs)` per axis, relative to
//! the lattice origin):
//!
//! 1. **In-band cells** (allocated tiles, all eight corner samples strictly
//!    inside ±`band`): trilinear interpolation of cached samples stored as
//!    16-bit fixed point over `[−band, band]` — quantum `band/32767`
//!    (6.1e-5 mm for `band` = 2 mm), ~30× below the trilinear method error
//!    `O(vs²·curvature)`. Every surface-straddling cell is such a cell when
//!    `band ≥ 2·vs` (its corner `|d| ≤ √3·vs < band`), which `build`
//!    enforces by clamping. 16-bit in-band storage is a deliberate design
//!    decision: it is what brings the pinned r = 80 mm / 0.4 mm / ±2 mm case
//!    under 5 % of the dense grid — f32 tiles cannot get below ~8 % there
//!    because the ±2 mm shell alone is 4 % of the domain (arithmetic in
//!    `tests/sparse.rs`).
//! 2. **Unallocated tiles**: `distance` returns the tile's constant far
//!    value, which STRICTLY under-claims: `|value| ≤ |d(p)|` at every point
//!    `p` of the stride cube, sign-correct. Proof: a never-sampled tile
//!    stores `sign(d(c))·(|d(c)| − h)` with `h` the stride-cube
//!    half-diagonal, and `|d(p)| ≥ |d(c)| − h`; a sampled-then-dropped tile
//!    (all 512 samples out of band) stores `sign·(min|s| − √3·vs)`, and any
//!    stride-cube point is within `√3·vs` of one of its own samples.
//! 3. **Boundary cells** of allocated tiles with a corner in an unallocated
//!    tile mix cached samples with that neighbour's far constant:
//!    sign-correct (Lipschitz makes mixed-sign corners impossible there —
//!    any corner with `|d| < band` is always cached exactly), magnitude
//!    within one cell diagonal of an under-claim. Such cells only exist
//!    where `|d| ≥ band − √3·vs`, i.e. never at the surface.
//! 4. **Outside the lattice**: the query is clamped to the lattice box and
//!    the exterior gap is added ([`crate::grid::VoxelGrid`] convention) — a
//!    sensible positive far value when the surface is inside the built
//!    bounds, but NOT covered by the conservativeness claim above.
//!
//! Global safety property used by meshers and pruners: **the field never
//! claims more distance than the trilinear interpolant of exact samples
//! would (+½ quantum)** — every far substitution only lowers magnitudes. Any
//! Lipschitz-margin consumer (narrow-band seeding, sphere tracing) is at
//! least as safe on this cache as on a dense trilinear cache of the same
//! lattice, and zones 2–3 never carry a sign change, so meshing through the
//! cache reproduces the dense-cache surface (pinned: watertight volume
//! parity against the analytic field).
//!
//! The result is a valid distance *bound*, not an exact SDF — wrap it as a
//! leaf with [`crate::Node::primitive_bound`] (same honest tagging as
//! `MeshSdf` / `Tpms`).
//!
//! # `OctreeGrid` scope — evaluation cache, NOT a seam-free meshing field
//!
//! Leaves store their eight corner samples; `distance` walks to the
//! containing leaf and interpolates trilinearly. Two prominent caveats:
//!
//! - **Cross-depth T-junctions**: neighbouring leaves of different depth
//!   interpolate their own corners, so values on a shared face need not
//!   agree — the field is DISCONTINUOUS at depth transitions. Both sides
//!   keep one sign there (transitions only happen away from the surface, see
//!   the allocation proof), so no spurious zero crossings arise, but do not
//!   run a dual mesher across coarse/fine boundaries and expect closed
//!   quads. For meshing through a cache use [`SparseGrid`] (uniform lattice).
//! - **Far-field magnitude is not conservative**: trilinear interpolation
//!   over a coarse far leaf has `O(cell²·curvature)` error in BOTH
//!   directions (for a convex body, Jensen's inequality makes the corner
//!   interpolant over-claim), so the coarse region must not drive
//!   narrow-band pruning or sphere tracing. What IS true and pinned:
//!   sign-correctness everywhere outside the trilinear error layer of
//!   max-depth leaves, and trilinear-class accuracy near the surface where
//!   depth is maximal.
//!
//! # Determinism (docs/NUMERICS.md contract)
//!
//! Identical inputs produce bit-identical caches: tiles are built by an
//! index-ordered `rayon` map whose collect preserves order, every tile's
//! content is a pure function of `(sdf, tile index)`, assembly is a serial
//! in-order pass, the octree is a serial preorder DFS with fixed octant
//! order, and this module contains no hash-map iteration at all. Pinned by
//! `content_hash()` equality across independent builds in `tests/sparse.rs`.

use rayon::prelude::*;

use kernel_core::math::{Aabb, Vec3};
use kernel_core::sdf::Sdf;

/// Samples per tile edge (8³ samples per tile).
const TILE: usize = 8;
/// Samples per tile.
const TILE3: usize = TILE * TILE * TILE;
/// Directory sentinel: tile not allocated (far).
const UNALLOCATED: u32 = u32::MAX;
/// Hard cap on the tile directory (2²⁴ tiles ⇒ ≤ 128 MB of directory + far
/// values). Larger requests are built at a uniformly coarsened voxel so the
/// whole domain stays covered — honest degradation, reported via `voxel()`.
const MAX_TILES: f64 = (1u64 << 24) as f64;
/// Node budget for [`OctreeGrid`] (2²⁴ nodes ≈ 600 MB): a pathological
/// all-band field stops refining instead of exhausting memory;
/// `max_depth_reached()` then reports the truth.
const MAX_OCTREE_NODES: usize = 1 << 24;

/// Trilinear interpolation of unit-cube corner values `c[g]`, `g = x | y<<1 | z<<2`
/// (same convention as `crate::grid`).
#[inline]
fn trilerp(c: [f32; 8], f: Vec3) -> f32 {
	let c00 = c[0] * (1.0 - f.x) + c[1] * f.x;
	let c10 = c[2] * (1.0 - f.x) + c[3] * f.x;
	let c01 = c[4] * (1.0 - f.x) + c[5] * f.x;
	let c11 = c[6] * (1.0 - f.x) + c[7] * f.x;
	let c0 = c00 * (1.0 - f.y) + c10 * f.y;
	let c1 = c01 * (1.0 - f.y) + c11 * f.y;
	c0 * (1.0 - f.z) + c1 * f.z
}

/// NaN-poisoned samples are treated as "far outside" (+∞) instead of being
/// silently cast to 0 (= "on the surface") by the fixed-point conversion.
#[inline]
fn finite_or_inf(d: f32) -> f32 {
	if d.is_nan() {
		f32::INFINITY
	} else {
		d
	}
}

/// Deterministic sign convention: `d < 0` → −1, else +1 (0 counts as outside,
/// matching the meshers' corner-mask convention).
#[inline]
fn sign_of(d: f32) -> f32 {
	if d < 0.0 {
		-1.0
	} else {
		1.0
	}
}

/// In-band samples are stored as 16-bit fixed point over `[−band, band]`:
/// quantum `band / 32767` (≤ 6.2e-5·band), ~30× below trilinear error.
#[inline]
fn quantize(d: f32, band: f32) -> i16 {
	((d.clamp(-band, band) / band) * 32767.0).round() as i16
}

#[inline]
fn dequantize(q: i16, band: f32) -> f32 {
	f32::from(q) / 32767.0 * band
}

/// FNV-1a 64 over raw little-endian bytes — dependency-free content hashing
/// for the determinism pins.
struct Fnv(u64);

impl Fnv {
	fn new() -> Self {
		Fnv(0xcbf2_9ce4_8422_2325)
	}
	fn write(&mut self, bytes: &[u8]) {
		for &b in bytes {
			self.0 ^= u64::from(b);
			self.0 = self.0.wrapping_mul(0x100_0000_01b3);
		}
	}
	fn write_u32(&mut self, v: u32) {
		self.write(&v.to_le_bytes());
	}
	fn write_f32(&mut self, v: f32) {
		self.write(&v.to_bits().to_le_bytes());
	}
}

/// Lattice-point counts (padded like the meshers: +3 points, min side padded
/// by one voxel, then rounded up to whole tiles), origin, and the effective
/// voxel. Mirrors `crate::grid::lattice_layout`, except an over-large domain
/// coarsens the voxel uniformly (full coverage kept) instead of truncating.
/// All count arithmetic is f64 so a runaway `size/voxel` cannot overflow.
fn tile_layout(domain: Aabb, voxel: f32) -> ([usize; 3], Vec3, f32) {
	let mut vs = if voxel.is_finite() { voxel.max(1e-6) } else { 1.0 };
	let size = domain.size();
	loop {
		let count = |extent: f32| -> f64 {
			let c = f64::from(extent) / f64::from(vs);
			if c.is_finite() && c >= 1.0 {
				c.ceil()
			} else {
				1.0
			}
		};
		let dims_f = [count(size.x), count(size.y), count(size.z)].map(|c| ((c + 3.0) / TILE as f64).ceil() * TILE as f64);
		let tiles = (dims_f[0] / TILE as f64) * (dims_f[1] / TILE as f64) * (dims_f[2] / TILE as f64);
		if tiles <= MAX_TILES {
			let dims = dims_f.map(|d| d as usize);
			return (dims, domain.min - Vec3::splat(vs), vs);
		}
		vs *= 1.26; // ≈ ∛2: halves the tile count per iteration ⇒ terminates
	}
}

// --- SparseGrid ---------------------------------------------------------------

/// Two-level block-sparse signed-distance cache: a coarse directory of
/// 8³-sample tiles, allocated only where the Lipschitz centre test (module
/// docs) says the ±`band` shell can be. Memory and build evaluations scale
/// with the surface band, not the domain volume — unlike
/// [`crate::grid::SparseGrid`], which evaluates the full dense lattice and is
/// only *storage*-sparse.
///
/// Field contract, safety proof, and the 16-bit in-band storage rationale:
/// module docs. Wrap as a CSG leaf with [`crate::Node::primitive_bound`].
#[derive(Clone, Debug)]
pub struct SparseGrid {
	origin: Vec3,
	voxel: f32,
	band: f32,
	/// Lattice-point counts per axis (whole multiples of the tile edge).
	dims: [usize; 3],
	tile_dims: [usize; 3],
	/// Per tile: slot index into `tiles`, or [`UNALLOCATED`].
	directory: Vec<u32>,
	/// Allocated tiles' 8³ samples, 16-bit fixed point over `[−band, band]`,
	/// concatenated in ascending tile order (deterministic).
	tiles: Vec<i16>,
	/// Per tile: the conservative far constant (meaningful when unallocated).
	far: Vec<f32>,
}

impl SparseGrid {
	/// Build a sparse cache of `sdf` over `bounds` at `voxel` resolution,
	/// keeping exact (16-bit fixed point) samples inside the ±`band` shell of
	/// the surface (world units; clamped to ≥ 2·voxel so every
	/// surface-straddling cell is fully backed by cached samples).
	///
	/// Allocation is the Lipschitz-safe centre test from the module docs
	/// (`|d(centre)| ≤ band + tile half-diagonal` — cannot miss a
	/// surface-crossing tile for any non-over-claiming field), followed by an
	/// exact refinement: a centre-passing tile whose 512 samples all fall
	/// outside ±`band` is provably surface-free and is demoted to a far
	/// constant derived from its own sampled minimum. Far tiles cost one
	/// evaluation each; allocated-candidate tiles cost 512.
	///
	/// Deterministic: identical inputs give bit-identical buffers (module
	/// docs). NaN field values are treated as +∞ (far outside). Domains
	/// needing more than 2²⁴ tiles are built at a uniformly coarsened voxel
	/// (`voxel()` reports the effective value).
	pub fn build<S: Sdf + ?Sized>(sdf: &S, bounds: Aabb, voxel: f32, band: f32) -> SparseGrid {
		let (dims, origin, vs) = tile_layout(bounds, voxel);
		let band = if band.is_finite() { band.max(2.0 * vs) } else { 2.0 * vs };
		let tile_dims = [dims[0] / TILE, dims[1] / TILE, dims[2] / TILE];
		let ntiles = tile_dims[0] * tile_dims[1] * tile_dims[2];
		// Stride-cube half-diagonal (covers all 512 sample sites AND the gap
		// cells up to the next tile's first sample plane).
		let h_tile = 0.5 * (TILE as f32) * vs * 3.0f32.sqrt();
		// Covering radius of the stride cube by the tile's own samples: the
		// farthest stride-cube point sits one voxel past the last sample plane
		// on each axis.
		let h_cell = vs * 3.0f32.sqrt();

		// Index-ordered parallel map; collect preserves order and each tile's
		// content is a pure function of its index ⇒ deterministic.
		let per_tile: Vec<(Option<Box<[i16; TILE3]>>, f32)> = (0..ntiles)
			.into_par_iter()
			.map(|t| {
				let tx = t % tile_dims[0];
				let ty = (t / tile_dims[0]) % tile_dims[1];
				let tz = t / (tile_dims[0] * tile_dims[1]);
				let base = [tx * TILE, ty * TILE, tz * TILE];
				// The stride-cube centre is itself a lattice point.
				let centre = origin + Vec3::new((base[0] + TILE / 2) as f32, (base[1] + TILE / 2) as f32, (base[2] + TILE / 2) as f32) * vs;
				let dc = finite_or_inf(sdf.distance(centre));
				if dc.abs() > band + h_tile {
					// Lipschitz-pruned: the whole stride cube is provably
					// > band from the surface; |d(p)| ≥ |dc| − h_tile there,
					// so this far constant strictly under-claims.
					return (None, sign_of(dc) * (dc.abs() - h_tile));
				}
				// Candidate: sample all 512 lattice sites (fixed z-y-x order).
				let mut samples = [0.0f32; TILE3];
				let mut min_abs = f32::INFINITY;
				let mut idx = 0;
				for lz in 0..TILE {
					for ly in 0..TILE {
						for lx in 0..TILE {
							let p = origin + Vec3::new((base[0] + lx) as f32, (base[1] + ly) as f32, (base[2] + lz) as f32) * vs;
							let d = finite_or_inf(sdf.distance(p));
							samples[idx] = d;
							min_abs = min_abs.min(d.abs());
							idx += 1;
						}
					}
				}
				if min_abs < band {
					let mut q = Box::new([0i16; TILE3]);
					for (qi, s) in q.iter_mut().zip(samples.iter()) {
						*qi = quantize(*s, band);
					}
					(Some(q), 0.0)
				} else {
					// Exact refinement: every sample is out of band ⇒ the tile
					// is surface-free and single-signed (adjacent samples are
					// ≤ √3·vs apart — an in-tile sign flip would need a jump
					// ≥ 2·band > √3·vs). Every stride-cube point is within
					// √3·vs of one of the tile's own samples, so this far
					// constant strictly under-claims too.
					(None, sign_of(samples[0]) * (min_abs - h_cell))
				}
			})
			.collect();

		let allocated = per_tile.iter().filter(|(d, _)| d.is_some()).count();
		let mut directory = vec![UNALLOCATED; ntiles];
		let mut far = vec![0.0f32; ntiles];
		let mut tiles: Vec<i16> = Vec::with_capacity(allocated * TILE3);
		for (t, (data, f)) in per_tile.into_iter().enumerate() {
			match data {
				Some(q) => {
					directory[t] = (tiles.len() / TILE3) as u32;
					tiles.extend_from_slice(&q[..]);
				}
				None => far[t] = f,
			}
		}
		SparseGrid { origin, voxel: vs, band, dims, tile_dims, directory, tiles, far }
	}

	/// World position of lattice point `(i, j, k)`.
	#[inline]
	fn point(&self, i: usize, j: usize, k: usize) -> Vec3 {
		self.origin + Vec3::new(i as f32, j as f32, k as f32) * self.voxel
	}

	#[inline]
	fn tile_index(&self, tx: usize, ty: usize, tz: usize) -> usize {
		tx + self.tile_dims[0] * (ty + self.tile_dims[1] * tz)
	}

	/// Field value at lattice point `(i, j, k)`: the cached sample if its
	/// owning tile is allocated, else that tile's conservative far constant.
	#[inline]
	fn value(&self, i: usize, j: usize, k: usize) -> f32 {
		let t = self.tile_index(i / TILE, j / TILE, k / TILE);
		match self.directory[t] {
			UNALLOCATED => self.far[t],
			slot => {
				let local = (i % TILE) + TILE * ((j % TILE) + TILE * (k % TILE));
				dequantize(self.tiles[slot as usize * TILE3 + local], self.band)
			}
		}
	}

	/// Heap payload of the cache: directory + far constants + 16-bit sample
	/// tiles + the struct header. Length-based (buffers are built to exact
	/// size), excluding allocator slack.
	pub fn memory_bytes(&self) -> usize {
		std::mem::size_of::<Self>()
			+ self.directory.len() * std::mem::size_of::<u32>()
			+ self.far.len() * std::mem::size_of::<f32>()
			+ self.tiles.len() * std::mem::size_of::<i16>()
	}

	/// Number of tiles storing dense 8³ sample data.
	pub fn allocated_tiles(&self) -> usize {
		self.tiles.len() / TILE3
	}

	/// Total tiles in the directory (allocated + far).
	pub fn total_tiles(&self) -> usize {
		self.directory.len()
	}

	/// Effective voxel size (may be coarser than requested if the domain
	/// exceeded the 2²⁴-tile directory cap — see [`SparseGrid::build`]).
	pub fn voxel(&self) -> f32 {
		self.voxel
	}

	/// Half-width of the exactly-cached shell around the surface (world
	/// units; ≥ 2·voxel by construction).
	pub fn band(&self) -> f32 {
		self.band
	}

	/// Lattice origin (min corner of the padded sample lattice).
	pub fn origin(&self) -> Vec3 {
		self.origin
	}

	/// Lattice-point counts per axis (whole multiples of `tile_samples()`).
	pub fn sample_dims(&self) -> [usize; 3] {
		self.dims
	}

	/// Tile counts per axis.
	pub fn tile_dims(&self) -> [usize; 3] {
		self.tile_dims
	}

	/// Samples per tile edge (the 8 of "8³-sample tiles").
	pub fn tile_samples(&self) -> usize {
		TILE
	}

	/// World edge length of one tile's stride cube (`tile_samples() · voxel()`).
	pub fn tile_size(&self) -> f32 {
		TILE as f32 * self.voxel
	}

	/// Whether tile `(tx, ty, tz)` stores dense samples (out-of-range → false).
	/// Inspection hook for allocation audits (used by the pinned negative
	/// control against over-claiming fields).
	pub fn tile_allocated(&self, tx: usize, ty: usize, tz: usize) -> bool {
		if tx >= self.tile_dims[0] || ty >= self.tile_dims[1] || tz >= self.tile_dims[2] {
			return false;
		}
		self.directory[self.tile_index(tx, ty, tz)] != UNALLOCATED
	}

	/// FNV-1a 64 over the complete cache content (layout metadata, directory,
	/// sample tiles, far constants). Two builds from identical inputs hash
	/// equal — the determinism pin (docs/NUMERICS.md).
	pub fn content_hash(&self) -> u64 {
		let mut h = Fnv::new();
		for d in self.dims {
			h.write(&(d as u64).to_le_bytes());
		}
		h.write_f32(self.voxel);
		h.write_f32(self.band);
		for c in [self.origin.x, self.origin.y, self.origin.z] {
			h.write_f32(c);
		}
		for &s in &self.directory {
			h.write_u32(s);
		}
		for &q in &self.tiles {
			h.write(&q.to_le_bytes());
		}
		for &f in &self.far {
			h.write_f32(f);
		}
		h.0
	}
}

impl Sdf for SparseGrid {
	/// Zone semantics (module docs): unallocated tile → its strict
	/// under-claiming far constant; allocated tile → trilinear over the 8³
	/// cached samples (corners falling in unallocated neighbours use that
	/// neighbour's far constant). Queries outside the lattice are clamped and
	/// the exterior gap added (`crate::grid` convention — not covered by the
	/// conservativeness guarantee).
	fn distance(&self, p: Vec3) -> f32 {
		let [nx, ny, nz] = self.dims;
		if nx < 2 || ny < 2 || nz < 2 {
			return f32::INFINITY;
		}
		let lo = self.origin;
		let hi = self.point(nx - 1, ny - 1, nz - 1);
		let clamped = p.clamp(lo, hi);
		let gap = (p - clamped).length();
		let l = (clamped - lo) / self.voxel;
		let i = (l.x.floor() as usize).min(nx - 2);
		let j = (l.y.floor() as usize).min(ny - 2);
		let k = (l.z.floor() as usize).min(nz - 2);
		let t = self.tile_index(i / TILE, j / TILE, k / TILE);
		if self.directory[t] == UNALLOCATED {
			return self.far[t] + gap;
		}
		let f = l - Vec3::new(i as f32, j as f32, k as f32);
		let c = [
			self.value(i, j, k),
			self.value(i + 1, j, k),
			self.value(i, j + 1, k),
			self.value(i + 1, j + 1, k),
			self.value(i, j, k + 1),
			self.value(i + 1, j, k + 1),
			self.value(i, j + 1, k + 1),
			self.value(i + 1, j + 1, k + 1),
		];
		trilerp(c, f) + gap
	}

	fn bounds(&self) -> Aabb {
		let [nx, ny, nz] = self.dims;
		Aabb::new(self.origin, self.point(nx - 1, ny - 1, nz - 1))
	}
}

// --- OctreeGrid ----------------------------------------------------------------

/// Sentinel: node has no children (leaf).
const LEAF: u32 = u32::MAX;

/// One octree node: index of the first of 8 contiguous children (or [`LEAF`])
/// plus the node's own corner samples in trilerp order (`g = x | y<<1 | z<<2`).
#[derive(Clone, Copy, Debug)]
struct OctNode {
	children: u32,
	corners: [f32; 8],
}

/// Adaptive-depth sampled-SDF cache: a cell subdivides only while the
/// Lipschitz centre test `|d(centre)| ≤ band + cell half-diagonal` says the
/// ±`band` shell can intersect it (module docs — recursive form of the
/// narrow-band coarse scan), so depth is maximal at the surface and coarse
/// far away.
///
/// **Scope: evaluation caching only.** Cross-depth neighbours are
/// discontinuous (T-junctions) and the coarse far field is not a
/// conservative bound — see the prominent module-doc caveats. Near-surface
/// evaluation accuracy and global sign behaviour are what is pinned.
#[derive(Clone, Debug)]
pub struct OctreeGrid {
	root_min: Vec3,
	root_size: Vec3,
	band: f32,
	max_depth: usize,
	deepest: usize,
	nodes: Vec<OctNode>,
}

/// Unit offset of cube corner / octant `g` (bits are the axis offsets).
#[inline]
fn corner_offset(g: usize) -> Vec3 {
	Vec3::new((g & 1) as f32, ((g >> 1) & 1) as f32, ((g >> 2) & 1) as f32)
}

fn sample_corners<S: Sdf + ?Sized>(sdf: &S, min: Vec3, size: Vec3) -> [f32; 8] {
	let mut c = [0.0f32; 8];
	for (g, v) in c.iter_mut().enumerate() {
		*v = finite_or_inf(sdf.distance(min + size * corner_offset(g)));
	}
	c
}

impl OctreeGrid {
	/// Build an adaptive cache of `sdf` over `bounds`, subdividing while the
	/// Lipschitz centre test keeps a cell in play and `max_depth` (clamped to
	/// 32 — beyond that cell sizes underflow f32) is not reached. Leaves store
	/// corner samples; see the type/module docs for the safety proof and the
	/// evaluation-caching scope. Serial preorder DFS with fixed octant order —
	/// deterministic; a 2²⁴-node budget stops refinement of pathological
	/// all-band fields instead of exhausting memory.
	pub fn build<S: Sdf + ?Sized>(sdf: &S, bounds: Aabb, max_depth: usize, band: f32) -> OctreeGrid {
		let max_depth = max_depth.min(32);
		let size = bounds.size();
		let valid = bounds.min.is_finite() && size.is_finite() && size.min_element() > 0.0;
		let (root_min, root_size) = if valid { (bounds.min, size) } else { (Vec3::ZERO, Vec3::ONE) };
		let band = if band.is_finite() { band.max(0.0) } else { 0.0 };
		let mut nodes = Vec::new();
		let root_corners = if valid { sample_corners(sdf, root_min, root_size) } else { [f32::INFINITY; 8] };
		nodes.push(OctNode { children: LEAF, corners: root_corners });
		let mut deepest = 0;
		if valid {
			subdivide(sdf, &mut nodes, 0, root_min, root_size, 0, max_depth, band, &mut deepest);
		}
		OctreeGrid { root_min, root_size, band, max_depth, deepest, nodes }
	}

	/// Total nodes in the tree (interior + leaves). Compare against the
	/// full-depth count `(8^(max_depth+1) − 1) / 7` for the sparsity ratio.
	pub fn node_count(&self) -> usize {
		self.nodes.len()
	}

	/// Deepest subdivision level actually reached (0 = root only). Equals the
	/// requested depth whenever the ±band shell is present and the node
	/// budget was not hit.
	pub fn max_depth_reached(&self) -> usize {
		self.deepest
	}

	/// Requested (clamped) maximum depth.
	pub fn max_depth(&self) -> usize {
		self.max_depth
	}

	/// Band half-width used by the subdivision test (world units).
	pub fn band(&self) -> f32 {
		self.band
	}

	/// Heap payload: node array + struct header (length-based).
	pub fn memory_bytes(&self) -> usize {
		std::mem::size_of::<Self>() + self.nodes.len() * std::mem::size_of::<OctNode>()
	}

	/// FNV-1a 64 over the complete tree content — the determinism pin.
	pub fn content_hash(&self) -> u64 {
		let mut h = Fnv::new();
		for c in [self.root_min.x, self.root_min.y, self.root_min.z, self.root_size.x, self.root_size.y, self.root_size.z, self.band] {
			h.write_f32(c);
		}
		h.write(&(self.max_depth as u64).to_le_bytes());
		h.write(&(self.deepest as u64).to_le_bytes());
		for n in &self.nodes {
			h.write_u32(n.children);
			for &c in &n.corners {
				h.write_f32(c);
			}
		}
		h.0
	}
}

/// Recursive Lipschitz-pruned subdivision (preorder DFS, fixed octant order).
/// The centre is the exact cell centre — no lattice-quantisation margin is
/// needed, unlike `narrow_band::find_seeds` whose probe snaps to a lattice
/// point.
#[allow(clippy::too_many_arguments)] // internal recursion state, not API
fn subdivide<S: Sdf + ?Sized>(
	sdf: &S,
	nodes: &mut Vec<OctNode>,
	node: usize,
	min: Vec3,
	size: Vec3,
	depth: usize,
	max_depth: usize,
	band: f32,
	deepest: &mut usize,
) {
	*deepest = (*deepest).max(depth);
	if depth >= max_depth || nodes.len() + 8 > MAX_OCTREE_NODES {
		return;
	}
	let centre = min + size * 0.5;
	let dc = finite_or_inf(sdf.distance(centre));
	let half_diag = (size * 0.5).length();
	if dc.abs() > band + half_diag {
		return; // provably no surface / band inside this cell (module docs)
	}
	let base = nodes.len();
	nodes[node].children = base as u32;
	let half = size * 0.5;
	// Push all 8 children first so they are contiguous, then recurse.
	for g in 0..8 {
		let cmin = min + half * corner_offset(g);
		let corners = sample_corners(sdf, cmin, half);
		nodes.push(OctNode { children: LEAF, corners });
	}
	for g in 0..8 {
		let cmin = min + half * corner_offset(g);
		subdivide(sdf, nodes, base + g, cmin, half, depth + 1, max_depth, band, deepest);
	}
}

impl Sdf for OctreeGrid {
	/// Walks to the leaf containing `p` (clamped into the root cell; the
	/// exterior gap is added back, `crate::grid` convention) and interpolates
	/// that leaf's corner samples trilinearly. Discontinuous across
	/// different-depth neighbours — see the module-doc caveats.
	fn distance(&self, p: Vec3) -> f32 {
		let lo = self.root_min;
		let hi = self.root_min + self.root_size;
		let q = p.clamp(lo, hi);
		let gap = (p - q).length();
		let mut idx = 0usize;
		let mut min = lo;
		let mut size = self.root_size;
		while self.nodes[idx].children != LEAF {
			let centre = min + size * 0.5;
			let mut g = 0usize;
			if q.x >= centre.x {
				g |= 1;
				min.x = centre.x;
			}
			if q.y >= centre.y {
				g |= 2;
				min.y = centre.y;
			}
			if q.z >= centre.z {
				g |= 4;
				min.z = centre.z;
			}
			size *= 0.5;
			idx = self.nodes[idx].children as usize + g;
		}
		let f = ((q - min) / size).clamp(Vec3::ZERO, Vec3::ONE);
		trilerp(self.nodes[idx].corners, f) + gap
	}

	fn bounds(&self) -> Aabb {
		Aabb::new(self.root_min, self.root_min + self.root_size)
	}
}
