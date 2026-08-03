// Copyright (c) LMCAD. Licensed under the MIT License.

//! Surface Nets — the MVP mesher.
//!
//! One vertex per surface-straddling cell, placed at the average of the cell's
//! edge zero-crossings; quads stitched across every sign-changing grid edge.
//! The result is watertight, smooth, and manifold. Sharp features are rounded —
//! that is the job of Dual Contouring (a later upgrade) which reuses the same
//! grid sampling but solves a QEF from Hermite data.
//!
//! Naive Surface Nets after Gibson / S. Lysenko, ported to operate over any
//! [`Sdf`] sampled on a regular lattice.

use rayon::prelude::*;

use crate::math::{Aabb, Vec3};
use crate::mesh::Mesh;
use crate::sdf::Sdf;

/// Maximum lattice points a mesher will allocate (≈ 268 M ≈ 1 GB of `f32`). A
/// finite-but-huge `domain.size() / voxel_size` ratio is refused (empty mesh)
/// rather than overflowing the `usize` cast / the `nx*ny*nz` allocation.
pub const MAX_LATTICE_CELLS: f64 = (1u64 << 28) as f64;

/// Sampling resolution for the mesher.
#[derive(Clone, Copy, Debug)]
pub enum Resolution {
	/// Cube edge length in world units.
	VoxelSize(f32),
	/// Target number of cells along the box's longest axis.
	CellsOnLongestAxis(u32),
}

impl Resolution {
	/// Resolve the concrete voxel size for a given domain.
	pub fn voxel_size(self, domain: Aabb) -> f32 {
		match self {
			// Do NOT launder a non-finite / non-positive size into a tiny 1e-6 voxel —
			// that would blow the grid up to billions of cells and overflow. Return NaN,
			// which the mesher's domain guard (and saturating float→int casts) turn into
			// an empty mesh.
			Resolution::VoxelSize(v) if v.is_finite() && v > 0.0 => v,
			Resolution::VoxelSize(_) => f32::NAN,
			Resolution::CellsOnLongestAxis(n) => {
				let longest = domain.size().max_element().max(1e-6);
				longest / (n.max(1) as f32)
			}
		}
	}
}

impl From<f32> for Resolution {
	fn from(v: f32) -> Self {
		Resolution::VoxelSize(v)
	}
}

/// Mesh any [`Sdf`] over `domain` using Surface Nets.
///
/// One cell of padding is added on every side so surfaces that reach the
/// domain boundary still close. Vertex normals come from the exact SDF
/// gradient; winding is corrected to outward (positive volume).
///
/// **Manifold guarantee:** the output is always a *closed* surface (no boundary
/// edges) and is fully 2-manifold when the surface is adequately resolved.
/// Because naive Surface Nets places a single vertex per cell, a cell straddled
/// by two surface sheets (a sub-voxel feature, or two near-tangent solids) can
/// leave a few non-manifold *edges*; mesh at a finer `resolution` relative to
/// the smallest feature to avoid this (a future Manifold Dual Contouring pass
/// would remove the limitation entirely).
pub fn surface_nets<S>(sdf: &S, domain: Aabb, resolution: impl Into<Resolution>) -> Mesh
where
	S: Sdf + ?Sized + Sync,
{
	let vs = resolution.into().voxel_size(domain);
	let size = domain.size();

	// Reject unmeshable domains: non-finite extents (e.g. a bare half-space), a
	// degenerate/inverted box (e.g. an empty CSG intersection), or a bad voxel size.
	if !domain.min.is_finite() || !domain.max.is_finite() || size.min_element() <= 0.0 || !vs.is_finite() || vs <= 0.0 {
		return Mesh::new();
	}

	// Sample-point counts per axis: enough to span the domain, plus padding so
	// the iso-surface is closed even when it touches the domain bounds. Guard the
	// lattice size in f64 BEFORE the usize cast: a finite-but-huge size/voxel ratio
	// would otherwise saturate the cast and overflow the `nx*ny*nz` allocation.
	let counts = [(size.x / vs).ceil(), (size.y / vs).ceil(), (size.z / vs).ceil()];
	let cells = (counts[0] as f64 + 3.0) * (counts[1] as f64 + 3.0) * (counts[2] as f64 + 3.0);
	if !(cells.is_finite() && cells <= MAX_LATTICE_CELLS) {
		return Mesh::new();
	}
	let nx = counts[0] as usize + 3;
	let ny = counts[1] as usize + 3;
	let nz = counts[2] as usize + 3;
	let dims = [nx, ny, nz];
	let origin = domain.min - Vec3::splat(vs); // one padding cell on the min side

	// Sample the field at every lattice point (parallel over z-slices).
	let mut data = vec![0f32; nx * ny * nz];
	let slice_stride = nx * ny;
	data.par_chunks_mut(slice_stride).enumerate().for_each(|(k, slice)| {
		for j in 0..ny {
			let base = nx * j;
			for i in 0..nx {
				let p = origin + Vec3::new(i as f32, j as f32, k as f32) * vs;
				slice[base + i] = sdf.distance(p);
			}
		}
	});

	march(sdf, &data, dims, origin, vs)
}

/// The Surface Nets march over a pre-sampled scalar field.
fn march<S>(sdf: &S, data: &[f32], dims: [usize; 3], origin: Vec3, vs: f32) -> Mesh
where
	S: Sdf + ?Sized + Sync,
{
	let (cube_edges, edge_table) = crate::marching::edge_tables();
	let [nx, ny, nz] = dims;
	let (cdx, cdy, cdz) = (nx.saturating_sub(1), ny.saturating_sub(1), nz.saturating_sub(1));
	if cdx == 0 || cdy == 0 || cdz == 0 {
		return Mesh::new();
	}
	let cell_stride = [1usize, cdx, cdx * cdy];
	let cell_count = cdx * cdy * cdz;
	let layer = cdx * cdy;
	let data_ref = data;

	// Phase A (parallel): place each cell's vertex (and sample its normal). Cells
	// are independent, so the per-cell work — including the SDF gradient — fans
	// out across cores. The averaged crossing position needs no SDF evaluation.
	let cell_data: Vec<Option<(Vec3, Vec3, u32)>> = (0..cell_count)
		.into_par_iter()
		.map(|ci| {
			let cz = ci / layer;
			let rem = ci - cz * layer;
			let cy = rem / cdx;
			let cx = rem - cy * cdx;

			let mut grid = [0f32; 8];
			let mut mask = 0u32;
			// `g` is a 3-bit cube-corner code (its bits ARE the offsets), not a position.
			#[allow(clippy::needless_range_loop)]
			for g in 0..8usize {
				let (oi, oj, ok) = (g & 1, (g >> 1) & 1, (g >> 2) & 1);
				let val = data_ref[(cx + oi) + nx * ((cy + oj) + ny * (cz + ok))];
				grid[g] = val;
				if val < 0.0 {
					mask |= 1 << g;
				}
			}
			if mask == 0 || mask == 0xff {
				return None;
			}

			let edge_mask = edge_table[mask as usize];
			let mut v = Vec3::ZERO;
			let mut e_count = 0.0f32;
			for e in 0..12usize {
				if edge_mask & (1 << e) == 0 {
					continue;
				}
				let c0 = cube_edges[e << 1];
				let c1 = cube_edges[(e << 1) + 1];
				let denom = grid[c0] - grid[c1];
				if denom.abs() < 1e-12 {
					continue;
				}
				let t = grid[c0] / denom; // zero crossing along c0 -> c1
				e_count += 1.0;
				for axis in 0..3usize {
					let bit = 1 << axis;
					let a = c0 & bit != 0;
					let b = c1 & bit != 0;
					if a != b {
						v[axis] += if a { 1.0 - t } else { t };
					} else if a {
						v[axis] += 1.0;
					}
				}
			}
			if e_count == 0.0 {
				return None;
			}
			v /= e_count;
			let world = origin + (Vec3::new(cx as f32, cy as f32, cz as f32) + v) * vs;
			Some((world, sdf.gradient(world), mask))
		})
		.collect();

	// Phase B (serial, cheap): assign vertex ids in cell order — identical order
	// and placement to the former serial march, so the output is unchanged.
	let mut mesh = Mesh::new();
	let mut cell_vertex = vec![u32::MAX; cell_count];
	for (ci, slot) in cell_data.iter().enumerate() {
		if let Some((w, n, _)) = *slot {
			cell_vertex[ci] = mesh.push_vertex(w);
			mesh.normals.push(n);
		}
	}

	// Phase C (serial, cheap): emit a quad for each straddling minimal edge,
	// joining the 4 cells around it.
	for (ci, slot) in cell_data.iter().enumerate() {
		let Some((_, _, mask)) = *slot else { continue };
		let cz = ci / layer;
		let rem = ci - cz * layer;
		let cy = rem / cdx;
		let cx = rem - cy * cdx;
		let edge_mask = edge_table[mask as usize];
		for axis in 0..3usize {
			if edge_mask & (1 << axis) == 0 {
				continue;
			}
			let iu = (axis + 1) % 3;
			let iv = (axis + 2) % 3;
			if [cx, cy, cz][iu] == 0 || [cx, cy, cz][iv] == 0 {
				continue; // a neighbour cell would be out of range
			}
			let (du, dv) = (cell_stride[iu], cell_stride[iv]);
			let q0 = cell_vertex[ci];
			let q1 = cell_vertex[ci - du];
			let q2 = cell_vertex[ci - du - dv];
			let q3 = cell_vertex[ci - dv];
			if q0 == u32::MAX || q1 == u32::MAX || q2 == u32::MAX || q3 == u32::MAX {
				continue;
			}
			// Orient the quad by whether corner 0 is inside.
			let (a, b, c, d) = if mask & 1 != 0 { (q0, q1, q2, q3) } else { (q0, q3, q2, q1) };
			mesh.push_triangle(a, b, c);
			mesh.push_triangle(a, c, d);
		}
	}

	mesh.ensure_outward();
	mesh
}
