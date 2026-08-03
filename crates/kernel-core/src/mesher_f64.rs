// Copyright (c) LMCAD. Licensed under the MIT License.

//! Full-`f64` Surface Nets.
//!
//! The standard [`mesher`](crate::mesher) samples on an `f32` grid, which loses
//! resolution for fine features held far from the origin (a 1 mm feature at a
//! 1 km offset falls below `f32`'s ~7 significant digits). This variant runs the
//! identical algorithm in `f64` over any field `Fn(DVec3) -> f64` — the analytic
//! surfaces and the implicit primitives all expose one — and emits an [`MeshF64`]
//! whose vertices keep `f64` precision end to end.

use rayon::prelude::*;

use crate::math::{Aabb, DMat3, DVec3};
use crate::mesh::Mesh;
use crate::mesher::MAX_LATTICE_CELLS;
use crate::sdf::Sdf;

/// A triangle mesh carrying `f64` vertex positions.
#[derive(Clone, Debug, Default)]
pub struct MeshF64 {
	pub positions: Vec<DVec3>,
	pub indices: Vec<u32>,
}

impl MeshF64 {
	/// Number of triangles.
	pub fn triangle_count(&self) -> usize {
		self.indices.len() / 3
	}

	/// Signed volume via the divergence theorem. Tetrahedra are taken to the vertex
	/// centroid, not the origin, so the sum stays well-conditioned even when the mesh
	/// sits far from the origin (origin-relative terms would cancel catastrophically).
	pub fn signed_volume(&self) -> f64 {
		if self.positions.is_empty() {
			return 0.0;
		}
		let c = self.positions.iter().fold(DVec3::ZERO, |s, p| s + *p) / self.positions.len() as f64;
		let mut v = 0.0;
		for t in self.indices.chunks_exact(3) {
			let a = self.positions[t[0] as usize] - c;
			let b = self.positions[t[1] as usize] - c;
			let d = self.positions[t[2] as usize] - c;
			v += a.dot(b.cross(d));
		}
		v / 6.0
	}

	/// Flip winding if the signed volume is negative (outward-facing result).
	fn ensure_outward(&mut self) {
		if self.signed_volume() < 0.0 {
			for t in self.indices.chunks_exact_mut(3) {
				t.swap(1, 2);
			}
		}
	}

	/// Down-cast to the `f32` [`Mesh`] (for export / GPU paths that need `f32`).
	pub fn to_mesh(&self) -> Mesh {
		let mut m = Mesh::new();
		m.positions = self.positions.iter().map(|p| p.as_vec3()).collect();
		m.indices = self.indices.clone();
		m
	}

	/// Wavefront OBJ text at full `f64` precision — vertex coordinates round-trip
	/// exactly (unlike the `f32` [`Mesh`] export, which truncates to ~7 digits).
	pub fn to_obj(&self) -> String {
		use std::fmt::Write;
		let f = |v: f64| if v.is_finite() { v } else { 0.0 };
		let mut s = String::new();
		for p in &self.positions {
			let _ = writeln!(s, "v {} {} {}", f(p.x), f(p.y), f(p.z));
		}
		for t in self.indices.chunks_exact(3) {
			let _ = writeln!(s, "f {} {} {}", t[0] + 1, t[1] + 1, t[2] + 1);
		}
		s
	}

	/// Binary STL bytes (the interchange/3D-print format). STL is `f32`, so positions
	/// are down-cast; for full `f64` precision use [`to_obj`](Self::to_obj) instead.
	pub fn to_stl_binary(&self) -> Vec<u8> {
		self.to_mesh().to_stl_binary()
	}
}

/// Surface Nets over an `f64` implicit field (negative inside) on the box
/// `[dmin, dmax]`. One padding cell is added per side so the surface closes at the
/// domain boundary; vertices are placed at the averaged edge crossings in `f64`.
///
/// ```
/// use kernel_core::{surface_nets_f64, MeshF64};
/// use kernel_core::math::DVec3;
/// // Mesh a unit sphere given as the implicit field f(p) = |p| − 1.
/// let m: MeshF64 = surface_nets_f64(|p| p.length() - 1.0, DVec3::splat(-1.5), DVec3::splat(1.5), 0.1);
/// assert!(m.triangle_count() > 100);
/// assert!((m.signed_volume() - 4.0 / 3.0 * std::f64::consts::PI).abs() < 0.1);
/// ```
pub fn surface_nets_f64<F>(field: F, dmin: DVec3, dmax: DVec3, voxel: f64) -> MeshF64
where
	F: Fn(DVec3) -> f64 + Sync,
{
	let size = dmax - dmin;
	if !dmin.is_finite() || !dmax.is_finite() || size.min_element() <= 0.0 || !voxel.is_finite() || voxel <= 0.0 {
		return MeshF64::default();
	}
	// Guard the lattice size in f64 before the usize cast (a huge size/voxel ratio
	// would otherwise overflow the allocation).
	let counts = [(size.x / voxel).ceil(), (size.y / voxel).ceil(), (size.z / voxel).ceil()];
	let cells = (counts[0] + 3.0) * (counts[1] + 3.0) * (counts[2] + 3.0);
	if !(cells.is_finite() && cells <= MAX_LATTICE_CELLS) {
		return MeshF64::default();
	}
	let (nx, ny, nz) = (counts[0] as usize + 3, counts[1] as usize + 3, counts[2] as usize + 3);
	let origin = dmin - DVec3::splat(voxel); // one padding cell on the min side

	// Sample the field at every lattice point (parallel over z-slices).
	let mut data = vec![0f64; nx * ny * nz];
	let slice = nx * ny;
	data.par_chunks_mut(slice).enumerate().for_each(|(k, row)| {
		for j in 0..ny {
			let base = nx * j;
			for i in 0..nx {
				row[base + i] = field(origin + DVec3::new(i as f64, j as f64, k as f64) * voxel);
			}
		}
	});

	march(&data, [nx, ny, nz], origin, voxel)
}

/// The Surface Nets march over a pre-sampled `f64` field (mirrors the `f32` march).
fn march(data: &[f64], dims: [usize; 3], origin: DVec3, vs: f64) -> MeshF64 {
	let (cube_edges, edge_table) = crate::marching::edge_tables();
	let [nx, ny, nz] = dims;
	let (cdx, cdy, cdz) = (nx.saturating_sub(1), ny.saturating_sub(1), nz.saturating_sub(1));
	if cdx == 0 || cdy == 0 || cdz == 0 {
		return MeshF64::default();
	}
	let cell_stride = [1usize, cdx, cdx * cdy];
	let cell_count = cdx * cdy * cdz;
	let layer = cdx * cdy;

	// Phase A (parallel): place each straddling cell's vertex at the averaged crossing.
	let cell_data: Vec<Option<(DVec3, u32)>> = (0..cell_count)
		.into_par_iter()
		.map(|ci| {
			let cz = ci / layer;
			let rem = ci - cz * layer;
			let cy = rem / cdx;
			let cx = rem - cy * cdx;

			let mut grid = [0f64; 8];
			let mut mask = 0u32;
			#[allow(clippy::needless_range_loop)]
			for g in 0..8usize {
				let (oi, oj, ok) = (g & 1, (g >> 1) & 1, (g >> 2) & 1);
				let val = data[(cx + oi) + nx * ((cy + oj) + ny * (cz + ok))];
				grid[g] = val;
				if val < 0.0 {
					mask |= 1 << g;
				}
			}
			if mask == 0 || mask == 0xff {
				return None;
			}
			let edge_mask = edge_table[mask as usize];
			let mut v = DVec3::ZERO;
			let mut e_count = 0.0f64;
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
				let t = grid[c0] / denom;
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
			let world = origin + (DVec3::new(cx as f64, cy as f64, cz as f64) + v) * vs;
			Some((world, mask))
		})
		.collect();

	// Phase B: assign vertex ids in cell order.
	let mut mesh = MeshF64::default();
	let mut cell_vertex = vec![u32::MAX; cell_count];
	for (ci, slot) in cell_data.iter().enumerate() {
		if let Some((w, _)) = *slot {
			cell_vertex[ci] = mesh.positions.len() as u32;
			mesh.positions.push(w);
		}
	}

	// Phase C: emit a quad for each straddling minimal edge.
	let mut tri = |a: u32, b: u32, c: u32| {
		mesh.indices.extend_from_slice(&[a, b, c]);
	};
	for (ci, slot) in cell_data.iter().enumerate() {
		let Some((_, mask)) = *slot else { continue };
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
			tri(a, b, c);
			tri(a, c, d);
		}
	}

	mesh.ensure_outward();
	mesh
}

/// Mesh any [`Sdf`] in full `f64` via Surface Nets, sampling its `distance64`. The
/// domain defaults to the SDF's own `bounds`; pass `voxel` in world units.
pub fn surface_nets_sdf_f64<S: Sdf + ?Sized + Sync>(sdf: &S, domain: Aabb, voxel: f64) -> MeshF64 {
	surface_nets_f64(|p| sdf.distance64(p), domain.min.as_dvec3(), domain.max.as_dvec3(), voxel)
}

/// Mesh any [`Sdf`] in full `f64` via Dual Contouring (sharp features preserved).
pub fn dual_contour_sdf_f64<S: Sdf + ?Sized + Sync>(sdf: &S, domain: Aabb, voxel: f64) -> MeshF64 {
	dual_contour_f64(|p| sdf.distance64(p), domain.min.as_dvec3(), domain.max.as_dvec3(), voxel)
}

/// Central-difference gradient of an `f64` field at `p` (step `h`).
fn gradient(field: &(impl Fn(DVec3) -> f64 + Sync), p: DVec3, h: f64) -> DVec3 {
	let dx = field(p + DVec3::X * h) - field(p - DVec3::X * h);
	let dy = field(p + DVec3::Y * h) - field(p - DVec3::Y * h);
	let dz = field(p + DVec3::Z * h) - field(p - DVec3::Z * h);
	(DVec3::new(dx, dy, dz) / (2.0 * h)).normalize_or_zero()
}

/// Refine an edge crossing of an `f64` field by a few bisection steps.
fn refine_crossing(field: &(impl Fn(DVec3) -> f64 + Sync), mut a: DVec3, mut b: DVec3, mut da: f64, mut db: f64) -> DVec3 {
	for _ in 0..6 {
		let t = if (da - db).abs() > 1e-12 { da / (da - db) } else { 0.5 };
		let m = a.lerp(b, t);
		let dm = field(m);
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

/// Solve the regularized QEF `min Σ (nᵢ·(x−pᵢ))² + λ|x−c|²`, clamped to the cell.
fn solve_qef(planes: &[(DVec3, DVec3)], centroid: DVec3, cell_min: DVec3, cell_max: DVec3) -> DVec3 {
	let mut ata = DMat3::ZERO;
	let mut atb = DVec3::ZERO;
	for &(p, n) in planes {
		ata += DMat3::from_cols(n * n.x, n * n.y, n * n.z);
		atb += n * n.dot(p);
	}
	let lambda = 0.10;
	let m = ata + DMat3::from_diagonal(DVec3::splat(lambda));
	let x = m.inverse() * (atb + centroid * lambda);
	x.clamp(cell_min, cell_max)
}

/// Full-`f64` Dual Contouring: like [`surface_nets_f64`] but each cell vertex is
/// placed by a QEF from the field's Hermite data (crossings + gradients), so sharp
/// edges and corners stay crisp instead of being rounded. Gradients come from
/// central differences of `field`.
pub fn dual_contour_f64<F>(field: F, dmin: DVec3, dmax: DVec3, voxel: f64) -> MeshF64
where
	F: Fn(DVec3) -> f64 + Sync,
{
	let size = dmax - dmin;
	if !dmin.is_finite() || !dmax.is_finite() || size.min_element() <= 0.0 || !voxel.is_finite() || voxel <= 0.0 {
		return MeshF64::default();
	}
	let counts = [(size.x / voxel).ceil(), (size.y / voxel).ceil(), (size.z / voxel).ceil()];
	let cells = (counts[0] + 3.0) * (counts[1] + 3.0) * (counts[2] + 3.0);
	if !(cells.is_finite() && cells <= MAX_LATTICE_CELLS) {
		return MeshF64::default();
	}
	let (nx, ny, nz) = (counts[0] as usize + 3, counts[1] as usize + 3, counts[2] as usize + 3);
	let origin = dmin - DVec3::splat(voxel);
	let h = voxel * 1e-3; // gradient step

	let mut data = vec![0f64; nx * ny * nz];
	let slice = nx * ny;
	data.par_chunks_mut(slice).enumerate().for_each(|(k, row)| {
		for j in 0..ny {
			let base = nx * j;
			for i in 0..nx {
				row[base + i] = field(origin + DVec3::new(i as f64, j as f64, k as f64) * voxel);
			}
		}
	});

	let (cube_edges, edge_table) = crate::marching::edge_tables();
	let (cdx, cdy, cdz) = (nx - 1, ny - 1, nz - 1);
	if cdx == 0 || cdy == 0 || cdz == 0 {
		return MeshF64::default();
	}
	let cell_stride = [1usize, cdx, cdx * cdy];
	let cell_count = cdx * cdy * cdz;
	let layer = cdx * cdy;
	let field = &field;

	let cell_data: Vec<Option<(DVec3, u32)>> = (0..cell_count)
		.into_par_iter()
		.map(|ci| {
			let cz = ci / layer;
			let rem = ci - cz * layer;
			let cy = rem / cdx;
			let cx = rem - cy * cdx;

			let mut grid = [0f64; 8];
			let mut mask = 0u32;
			#[allow(clippy::needless_range_loop)]
			for c in 0..8usize {
				let (oi, oj, ok) = (c & 1, (c >> 1) & 1, (c >> 2) & 1);
				let v = data[(cx + oi) + nx * ((cy + oj) + ny * (cz + ok))];
				grid[c] = v;
				if v < 0.0 {
					mask |= 1 << c;
				}
			}
			if mask == 0 || mask == 0xff {
				return None;
			}
			let cw = |c: usize| -> DVec3 {
				let (oi, oj, ok) = (c & 1, (c >> 1) & 1, (c >> 2) & 1);
				origin + DVec3::new((cx + oi) as f64, (cy + oj) as f64, (cz + ok) as f64) * voxel
			};
			let edge_mask = edge_table[mask as usize];
			let mut planes: Vec<(DVec3, DVec3)> = Vec::with_capacity(12);
			let mut centroid = DVec3::ZERO;
			for e in 0..12usize {
				if edge_mask & (1 << e) == 0 {
					continue;
				}
				let c0 = cube_edges[e << 1];
				let c1 = cube_edges[(e << 1) + 1];
				if (grid[c0] < 0.0) == (grid[c1] < 0.0) {
					continue;
				}
				let p = refine_crossing(field, cw(c0), cw(c1), grid[c0], grid[c1]);
				planes.push((p, gradient(field, p, h)));
				centroid += p;
			}
			if planes.is_empty() {
				return None;
			}
			centroid /= planes.len() as f64;
			let cell_min = cw(0);
			Some((solve_qef(&planes, centroid, cell_min, cell_min + DVec3::splat(voxel)), mask))
		})
		.collect();

	let mut mesh = MeshF64::default();
	let mut cell_vertex = vec![u32::MAX; cell_count];
	for (ci, slot) in cell_data.iter().enumerate() {
		if let Some((v, _)) = *slot {
			cell_vertex[ci] = mesh.positions.len() as u32;
			mesh.positions.push(v);
		}
	}

	for (ci, slot) in cell_data.iter().enumerate() {
		let Some((_, mask)) = *slot else { continue };
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
			mesh.indices.extend_from_slice(&[a, b, c, a, c, d]);
		}
	}

	mesh.ensure_outward();
	mesh
}
