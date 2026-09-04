// Copyright (c) LMCAD. Licensed under the MIT License.

//! Voxel grids that store a sampled **signed distance** (not occupancy), so they
//! can be combined, offset, and meshed like any other [`Sdf`].
//!
//! - [`VoxelGrid`]: dense `Vec<f32>` — trivial, ideal for an MVP and small parts.
//! - [`SparseGrid`]: hashed 8³ blocks within a narrow band — memory scales with
//!   surface area, the recommended production storage.
//!
//! Both implement [`Sdf`] via trilinear interpolation of the eight surrounding
//! lattice samples, so `kernel_core::surface_nets` (or any CSG `Node`) consumes
//! them directly. A grid is also a leaf: `Node::primitive(grid)`.

use std::collections::HashMap;

use rayon::prelude::*;

use kernel_core::math::{Aabb, Vec3};
use kernel_core::sdf::Sdf;

/// Trilinear interpolation of the unit cube corner values `c[xyz]`.
#[inline]
fn trilerp(c: [f32; 8], f: Vec3) -> f32 {
	let (fx, fy, fz) = (f.x, f.y, f.z);
	let c00 = c[0] * (1.0 - fx) + c[1] * fx;
	let c10 = c[2] * (1.0 - fx) + c[3] * fx;
	let c01 = c[4] * (1.0 - fx) + c[5] * fx;
	let c11 = c[6] * (1.0 - fx) + c[7] * fx;
	let c0 = c00 * (1.0 - fy) + c10 * fy;
	let c1 = c01 * (1.0 - fy) + c11 * fy;
	c0 * (1.0 - fz) + c1 * fz
}

/// Lattice sample counts and origin shared by the grids; padding ensures the
/// stored field brackets the surface on every side.
fn lattice_layout(domain: Aabb, vs: f32) -> ([usize; 3], Vec3) {
	let size = domain.size();
	let mut counts = [(size.x / vs).ceil(), (size.y / vs).ceil(), (size.z / vs).ceil()];
	// Bound the lattice in f64 before casting: a non-finite or runaway size/voxel
	// ratio would saturate the usize cast and overflow the allocation. If too large,
	// clamp the resolution down uniformly (coarser grid) rather than panic.
	for c in counts.iter_mut() {
		if !c.is_finite() || *c < 1.0 {
			*c = 1.0;
		}
	}
	let cells = (counts[0] as f64 + 3.0) * (counts[1] as f64 + 3.0) * (counts[2] as f64 + 3.0);
	if cells > kernel_core::mesher::MAX_LATTICE_CELLS {
		let scale = (kernel_core::mesher::MAX_LATTICE_CELLS / cells).cbrt() as f32;
		for c in counts.iter_mut() {
			*c = (*c * scale).floor().max(1.0);
		}
	}
	let nx = counts[0] as usize + 3;
	let ny = counts[1] as usize + 3;
	let nz = counts[2] as usize + 3;
	let origin = domain.min - Vec3::splat(vs);
	([nx, ny, nz], origin)
}

/// A dense signed-distance grid: `dims.0 * dims.1 * dims.2` samples in `data`.
#[derive(Clone, Debug)]
pub struct VoxelGrid {
	pub origin: Vec3,
	pub voxel_size: f32,
	pub dims: [usize; 3],
	pub data: Vec<f32>,
}

impl VoxelGrid {
	#[inline]
	fn index(&self, i: usize, j: usize, k: usize) -> usize {
		i + self.dims[0] * (j + self.dims[1] * k)
	}

	/// World position of lattice point `(i, j, k)`.
	pub fn point(&self, i: usize, j: usize, k: usize) -> Vec3 {
		self.origin + Vec3::new(i as f32, j as f32, k as f32) * self.voxel_size
	}

	/// Number of stored samples (dense ⇒ the full lattice).
	pub fn sample_count(&self) -> usize {
		self.data.len()
	}

	/// World-space box spanned by the lattice points.
	pub fn lattice_bounds(&self) -> Aabb {
		let max = self.point(self.dims[0] - 1, self.dims[1] - 1, self.dims[2] - 1);
		Aabb::new(self.origin, max)
	}

	/// Sample any [`Sdf`] onto a dense lattice covering `domain`.
	pub fn from_sdf<S>(sdf: &S, domain: Aabb, voxel_size: f32) -> Self
	where
		S: Sdf + ?Sized + Sync,
	{
		let vs = voxel_size.max(1e-6);
		let ([nx, ny, nz], origin) = lattice_layout(domain, vs);
		let mut data = vec![0.0f32; nx * ny * nz];
		data.par_chunks_mut(nx * ny).enumerate().for_each(|(k, slice)| {
			for j in 0..ny {
				let base = nx * j;
				for i in 0..nx {
					let p = origin + Vec3::new(i as f32, j as f32, k as f32) * vs;
					slice[base + i] = sdf.distance(p);
				}
			}
		});
		Self { origin, voxel_size: vs, dims: [nx, ny, nz], data }
	}
}

impl Sdf for VoxelGrid {
	fn distance(&self, p: Vec3) -> f32 {
		let [nx, ny, nz] = self.dims;
		// Trilinear interpolation needs a cell on every axis (dim >= 2); a degenerate
		// grid would underflow `nx - 2` and over-index `i + 1`. Bail gracefully.
		if nx < 2 || ny < 2 || nz < 2 {
			return self.data.first().copied().unwrap_or(f32::INFINITY);
		}
		let lo = self.origin;
		let hi = self.point(nx - 1, ny - 1, nz - 1);
		// Clamp the query into the lattice; account for the exterior gap so points
		// outside the grid still get a sensible (conservative) distance.
		let clamped = p.clamp(lo, hi);
		let l = (clamped - self.origin) / self.voxel_size;
		let i = (l.x.floor() as usize).min(nx - 2);
		let j = (l.y.floor() as usize).min(ny - 2);
		let k = (l.z.floor() as usize).min(nz - 2);
		let f = l - Vec3::new(i as f32, j as f32, k as f32);
		let c = [
			self.data[self.index(i, j, k)],
			self.data[self.index(i + 1, j, k)],
			self.data[self.index(i, j + 1, k)],
			self.data[self.index(i + 1, j + 1, k)],
			self.data[self.index(i, j, k + 1)],
			self.data[self.index(i + 1, j, k + 1)],
			self.data[self.index(i, j + 1, k + 1)],
			self.data[self.index(i + 1, j + 1, k + 1)],
		];
		trilerp(c, f) + (p - clamped).length()
	}

	fn bounds(&self) -> Aabb {
		self.lattice_bounds()
	}
}

// --- Sparse narrow-band grid -------------------------------------------------

/// Edge length (in lattice points) of a sparse storage block.
const BLOCK: usize = 8;
const BLOCK3: usize = BLOCK * BLOCK * BLOCK;

/// A hashed-block, narrow-band signed-distance grid.
///
/// Only blocks whose samples fall within the narrow band (`|d| <= band`) store
/// dense data; everywhere else a coarse per-block sign supplies a ±`band`
/// sentinel. Values are stored clamped to `±band`, so the field is continuous
/// across the allocated/unallocated boundary. Stored memory therefore scales
/// with surface area, not volume.
#[derive(Clone, Debug)]
pub struct SparseGrid {
	pub origin: Vec3,
	pub voxel_size: f32,
	pub band: f32,
	/// Lattice point counts (rounded up to a whole number of blocks).
	pub dims: [usize; 3],
	block_dims: [usize; 3],
	blocks: HashMap<usize, Box<[f32; BLOCK3]>>,
	/// Per-block coarse sign for unallocated blocks: `-1` inside, `+1` outside.
	block_sign: Vec<i8>,
}

impl SparseGrid {
	/// Sample any [`Sdf`] into a narrow-band sparse grid. `band_voxels` is the
	/// half-band width in voxels (≥ 2 recommended so trilerp neighbourhoods of
	/// every surface cell are fully resolved).
	pub fn from_sdf<S>(sdf: &S, domain: Aabb, voxel_size: f32, band_voxels: f32) -> Self
	where
		S: Sdf + ?Sized + Sync,
	{
		let vs = voxel_size.max(1e-6);
		let (raw_dims, origin) = lattice_layout(domain, vs);
		// Round point counts up to a whole number of blocks.
		let round = |n: usize| n.div_ceil(BLOCK) * BLOCK;
		let dims = [round(raw_dims[0]), round(raw_dims[1]), round(raw_dims[2])];
		let block_dims = [dims[0] / BLOCK, dims[1] / BLOCK, dims[2] / BLOCK];
		let band = band_voxels.max(1.0) * vs;
		let nblocks = block_dims[0] * block_dims[1] * block_dims[2];

		// Sample every block in parallel; keep dense data only near the surface.
		let per_block: Vec<(Option<Box<[f32; BLOCK3]>>, i8)> = (0..nblocks)
			.into_par_iter()
			.map(|b| {
				let bx = b % block_dims[0];
				let by = (b / block_dims[0]) % block_dims[1];
				let bz = b / (block_dims[0] * block_dims[1]);
				let mut buf = Box::new([0.0f32; BLOCK3]);
				let mut near = false;
				for lz in 0..BLOCK {
					for ly in 0..BLOCK {
						for lx in 0..BLOCK {
							let (i, j, k) = (bx * BLOCK + lx, by * BLOCK + ly, bz * BLOCK + lz);
							let p = origin + Vec3::new(i as f32, j as f32, k as f32) * vs;
							let d = sdf.distance(p).clamp(-band, band);
							buf[lx + BLOCK * (ly + BLOCK * lz)] = d;
							if d.abs() < band {
								near = true;
							}
						}
					}
				}
				// Coarse sign from the block centre (used only if unallocated).
				let center = origin
					+ Vec3::new((bx * BLOCK + BLOCK / 2) as f32, (by * BLOCK + BLOCK / 2) as f32, (bz * BLOCK + BLOCK / 2) as f32) * vs;
				let sign: i8 = if sdf.distance(center) < 0.0 { -1 } else { 1 };
				if near {
					(Some(buf), sign)
				} else {
					(None, sign)
				}
			})
			.collect();

		let mut blocks = HashMap::new();
		let mut block_sign = vec![1i8; nblocks];
		for (b, (data, sign)) in per_block.into_iter().enumerate() {
			block_sign[b] = sign;
			if let Some(buf) = data {
				blocks.insert(b, buf);
			}
		}

		Self { origin, voxel_size: vs, band, dims, block_dims, blocks, block_sign }
	}

	#[inline]
	fn block_index(&self, bx: usize, by: usize, bz: usize) -> usize {
		bx + self.block_dims[0] * (by + self.block_dims[1] * bz)
	}

	/// Signed value at lattice point `(i, j, k)` — stored data, or a ±band sentinel.
	#[inline]
	fn value(&self, i: usize, j: usize, k: usize) -> f32 {
		let b = self.block_index(i / BLOCK, j / BLOCK, k / BLOCK);
		match self.blocks.get(&b) {
			Some(buf) => {
				let (lx, ly, lz) = (i % BLOCK, j % BLOCK, k % BLOCK);
				buf[lx + BLOCK * (ly + BLOCK * lz)]
			}
			None => self.block_sign[b] as f32 * self.band,
		}
	}

	/// World position of lattice point `(i, j, k)`.
	pub fn point(&self, i: usize, j: usize, k: usize) -> Vec3 {
		self.origin + Vec3::new(i as f32, j as f32, k as f32) * self.voxel_size
	}

	/// Number of densely stored samples (allocated blocks × 8³).
	pub fn sample_count(&self) -> usize {
		self.blocks.len() * BLOCK3
	}

	/// Number of samples a dense grid of the same lattice would need.
	pub fn dense_sample_count(&self) -> usize {
		self.dims[0] * self.dims[1] * self.dims[2]
	}

	fn lattice_bounds(&self) -> Aabb {
		let max = self.point(self.dims[0] - 1, self.dims[1] - 1, self.dims[2] - 1);
		Aabb::new(self.origin, max)
	}
}

impl Sdf for SparseGrid {
	fn distance(&self, p: Vec3) -> f32 {
		let [nx, ny, nz] = self.dims;
		// A degenerate grid (any axis < 2 samples) cannot be trilinearly interpolated;
		// avoid the `nx - 2` underflow / `i + 1` over-index.
		if nx < 2 || ny < 2 || nz < 2 {
			return f32::INFINITY;
		}
		let lo = self.origin;
		let hi = self.point(nx - 1, ny - 1, nz - 1);
		let clamped = p.clamp(lo, hi);
		let l = (clamped - self.origin) / self.voxel_size;
		let i = (l.x.floor() as usize).min(nx - 2);
		let j = (l.y.floor() as usize).min(ny - 2);
		let k = (l.z.floor() as usize).min(nz - 2);
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
		trilerp(c, f) + (p - clamped).length()
	}

	fn bounds(&self) -> Aabb {
		self.lattice_bounds()
	}
}
