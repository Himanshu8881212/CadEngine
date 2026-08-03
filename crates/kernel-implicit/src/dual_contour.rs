// Copyright (c) LMCAD. Licensed under the MIT License.

//! Dual Contouring — the sharp-feature mesher.
//!
//! Same grid structure as Surface Nets, but each cell vertex is placed by
//! minimizing a quadratic error function (QEF) built from Hermite data: for
//! every zero-crossing edge, the crossing point and the **analytic SDF gradient**
//! there. Where normals disagree (an edge or corner) the QEF minimizer sits on
//! the sharp feature, so a knife trailing edge or a flat mating face stays crisp
//! instead of being rounded as Surface Nets would. A small Tikhonov term biased
//! to the cell centroid keeps the solve stable on flat/under-determined cells.

use rayon::prelude::*;

use kernel_core::marching::{edge_tables, CORNER_OFFSET};
use kernel_core::math::{Aabb, Mat3, Vec3};
use kernel_core::mesh::Mesh;
use kernel_core::mesher::Resolution;
use kernel_core::sdf::Sdf;

/// Solve the regularized QEF `min Σ (nᵢ·(x−pᵢ))² + λ|x−c|²`, clamped to the cell.
pub(crate) fn solve_qef(planes: &[(Vec3, Vec3)], centroid: Vec3, cell_min: Vec3, cell_max: Vec3) -> Vec3 {
	let mut ata = Mat3::ZERO;
	let mut atb = Vec3::ZERO;
	for &(p, n) in planes {
		// Outer product n nᵀ and rhs n (n·p).
		ata += Mat3::from_cols(n * n.x, n * n.y, n * n.z);
		atb += n * n.dot(p);
	}
	let lambda = 0.10;
	let m = ata + Mat3::from_diagonal(Vec3::splat(lambda));
	let rhs = atb + centroid * lambda;
	let x = m.inverse() * rhs;
	x.clamp(cell_min, cell_max)
}

/// Refine an edge zero-crossing with a few bisection steps for accuracy.
pub(crate) fn refine_crossing<S: Sdf + ?Sized>(sdf: &S, mut a: Vec3, mut b: Vec3, mut da: f32, mut db: f32) -> Vec3 {
	for _ in 0..4 {
		let t = if (da - db).abs() > 1e-12 { da / (da - db) } else { 0.5 };
		let m = a.lerp(b, t);
		let dm = sdf.distance(m);
		if (da < 0.0) == (dm < 0.0) {
			a = m;
			da = dm;
		} else {
			b = m;
			db = dm;
		}
	}
	let t = if (da - db).abs() > 1e-12 { da / (da - db) } else { 0.5 };
	a.lerp(b, t)
}

/// Mesh any [`Sdf`] over `domain` using Dual Contouring (sharp features preserved).
pub fn dual_contour<S>(sdf: &S, domain: Aabb, resolution: impl Into<Resolution>) -> Mesh
where
	S: Sdf + ?Sized + Sync,
{
	let vs = resolution.into().voxel_size(domain);
	let size = domain.size();
	if !domain.min.is_finite() || !domain.max.is_finite() || size.min_element() <= 0.0 || !vs.is_finite() || vs <= 0.0 {
		return Mesh::new();
	}
	let counts = [(size.x / vs).ceil(), (size.y / vs).ceil(), (size.z / vs).ceil()];
	let cells = (counts[0] as f64 + 3.0) * (counts[1] as f64 + 3.0) * (counts[2] as f64 + 3.0);
	if !(cells.is_finite() && cells <= kernel_core::mesher::MAX_LATTICE_CELLS) {
		return Mesh::new();
	}
	let nx = counts[0] as usize + 3;
	let ny = counts[1] as usize + 3;
	let nz = counts[2] as usize + 3;
	let origin = domain.min - Vec3::splat(vs);

	let mut data = vec![0f32; nx * ny * nz];
	data.par_chunks_mut(nx * ny).enumerate().for_each(|(k, slice)| {
		for j in 0..ny {
			let base = nx * j;
			for i in 0..nx {
				let p = origin + Vec3::new(i as f32, j as f32, k as f32) * vs;
				slice[base + i] = sdf.distance(p);
			}
		}
	});

	let (cube_edges, edge_table) = edge_tables();
	let (cdx, cdy, cdz) = (nx - 1, ny - 1, nz - 1);
	if cdx == 0 || cdy == 0 || cdz == 0 {
		return Mesh::new();
	}
	let cell_stride = [1usize, cdx, cdx * cdy];
	let cell_count = cdx * cdy * cdz;
	let layer = cdx * cdy;
	let data_ref = &data;

	// Phase A (parallel): the heavy per-cell work — Hermite sampling, gradients,
	// and the QEF solve — is independent per cell, so fan it out across cores.
	// Returns the placed vertex, its normal, and the cell mask (for face stitching).
	let cell_data: Vec<Option<(Vec3, Vec3, u32)>> = (0..cell_count)
		.into_par_iter()
		.map(|ci| {
			let cz = ci / layer;
			let rem = ci - cz * layer;
			let cy = rem / cdx;
			let cx = rem - cy * cdx;

			let mut grid = [0f32; 8];
			let mut mask = 0u32;
			for c in 0..8usize {
				let o = CORNER_OFFSET[c];
				let v = data_ref[(cx + o[0]) + nx * ((cy + o[1]) + ny * (cz + o[2]))];
				grid[c] = v;
				if v < 0.0 {
					mask |= 1 << c;
				}
			}
			if mask == 0 || mask == 0xff {
				return None;
			}
			let cw = |c: usize| -> Vec3 {
				let o = CORNER_OFFSET[c];
				origin + Vec3::new((cx + o[0]) as f32, (cy + o[1]) as f32, (cz + o[2]) as f32) * vs
			};

			let edge_mask = edge_table[mask as usize];
			let mut planes: Vec<(Vec3, Vec3)> = Vec::with_capacity(12);
			let mut centroid = Vec3::ZERO;
			for e in 0..12usize {
				if edge_mask & (1 << e) == 0 {
					continue;
				}
				let c0 = cube_edges[e << 1];
				let c1 = cube_edges[(e << 1) + 1];
				let (g0, g1) = (grid[c0], grid[c1]);
				if (g0 < 0.0) == (g1 < 0.0) {
					continue;
				}
				let p = refine_crossing(sdf, cw(c0), cw(c1), g0, g1);
				planes.push((p, sdf.gradient(p)));
				centroid += p;
			}
			if planes.is_empty() {
				return None;
			}
			centroid /= planes.len() as f32;
			let cell_min = cw(0);
			let vertex = solve_qef(&planes, centroid, cell_min, cell_min + Vec3::splat(vs));
			Some((vertex, sdf.gradient(vertex), mask))
		})
		.collect();

	// Phase B (serial, cheap): assign vertex ids in cell order — identical order
	// and placement to the old serial march, so output is unchanged.
	let mut mesh = Mesh::new();
	let mut cell_vertex = vec![u32::MAX; cell_count];
	for (ci, slot) in cell_data.iter().enumerate() {
		if let Some((v, n, _)) = *slot {
			cell_vertex[ci] = mesh.push_vertex(v);
			mesh.normals.push(n);
		}
	}

	// Phase C (serial, cheap): stitch a quad across each straddling minimal edge.
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
				continue;
			}
			let (du, dv) = (cell_stride[iu], cell_stride[iv]);
			let q0 = cell_vertex[ci];
			let q1 = cell_vertex[ci - du];
			let q2 = cell_vertex[ci - du - dv];
			let q3 = cell_vertex[ci - dv];
			if q0 == u32::MAX || q1 == u32::MAX || q2 == u32::MAX || q3 == u32::MAX {
				continue;
			}
			let (a, b, c, d) = if mask & 1 != 0 { (q0, q1, q2, q3) } else { (q0, q3, q2, q1) };
			mesh.push_triangle(a, b, c);
			mesh.push_triangle(a, c, d);
		}
	}

	mesh.ensure_outward();
	mesh
}
