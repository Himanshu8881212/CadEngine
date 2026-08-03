// Copyright (c) LMCAD. Licensed under the MIT License.

//! Narrow-band (surface-tracking) Surface Nets.
//!
//! Dense meshing samples and visits the whole `O(n³)` volume even though the
//! iso-surface is a 2-manifold whose cost should scale with surface **area**.
//! This module tracks the surface instead: it never samples the full grid up
//! front. It seeds from surface-straddling cells found by a Lipschitz-safe
//! coarse block scan (Lipschitz-safe: it never steps over the surface band),
//! then flood-fills across face-adjacent cells, retaining only cells that
//! straddle the zero level set. Corner SDF
//! samples are memoized in a [`HashMap`] keyed by lattice index so each lattice
//! point is evaluated at most once.
//!
//! Vertex placement and quad emission reuse the exact Surface Nets logic of
//! [`crate::surface_nets`], so the output is a watertight mesh equivalent to the
//! dense march — but with work proportional to the visited (active) cell count.
//!
//! ## Why flood-filling straddling cells is sufficient for watertightness
//!
//! Surface Nets emits a quad across every grid edge whose two endpoints have
//! mixed sign. The four cells incident to such an edge all contain that edge as
//! one of their twelve edges, so all four straddle and each owns a vertex.
//! Within those four cells, consecutive ones share a *face* (they differ by one
//! lattice step on a single axis), so they are reachable from one another by the
//! face-adjacent flood fill. Hence every cell that contributes a vertex to any
//! emitted quad is discovered, and the stitched surface is closed exactly as in
//! the dense march.

use std::collections::hash_map::Entry;
use std::collections::{HashMap, HashSet, VecDeque};

use kernel_core::marching::{edge_tables, CORNER_OFFSET};
use kernel_core::math::{Aabb, Vec3};
use kernel_core::mesh::Mesh;
use kernel_core::mesher::Resolution;
use kernel_core::sdf::Sdf;

use crate::dual_contour::{refine_crossing, solve_qef};

/// Which dual mesher the narrow-band march places vertices with.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Strategy {
	/// Surface Nets: one vertex at the average of edge crossings.
	SurfaceNets,
	/// Dual Contouring: QEF vertex from Hermite data (sharp features).
	DualContour,
}

/// Lattice geometry shared by the dense and narrow-band marches: padded sample
/// counts, the (min-side padded) origin, and the voxel size.
struct Lattice {
	dims: [usize; 3],
	origin: Vec3,
	vs: f32,
}

/// The narrow-band cell-count ceiling. The dense meshers cap the CONCEPTUAL lattice
/// at [`kernel_core::mesher::MAX_LATTICE_CELLS`] (2²⁸) because they allocate and
/// visit O(n³); the narrow band never materializes the grid — corner samples are
/// memoized per visited cell and work scales with surface AREA — so the conceptual
/// count only has to keep lattice point indices inside `usize` arithmetic. 2⁴⁴ allows
/// e.g. a 25 µm band over a 400 mm part while still leaving 2²⁰ headroom under
/// 64-bit indexing. (Before this dedicated cap, a fine voxel on a large domain
/// SILENTLY returned an empty mesh once the conceptual count crossed 2²⁸ — a
/// resin-resolution extraction of an Ø84 part at 0.12 mm hit exactly that.)
const NARROWBAND_MAX_LATTICE_CELLS: f64 = (1u64 << 44) as f64;

impl Lattice {
	/// Resolve the lattice for `domain` at the requested resolution, matching the
	/// padding (`+3` points, one padding cell on the min side) used by the dense
	/// Surface Nets mesher so the two produce identical grids.
	fn resolve(domain: Aabb, resolution: impl Into<Resolution>) -> Option<Self> {
		let vs = resolution.into().voxel_size(domain);
		let size = domain.size();
		if !domain.min.is_finite() || !domain.max.is_finite() || size.min_element() <= 0.0 || !vs.is_finite() || vs <= 0.0 {
			return None;
		}
		let counts = [(size.x / vs).ceil(), (size.y / vs).ceil(), (size.z / vs).ceil()];
		let cells = (counts[0] as f64 + 3.0) * (counts[1] as f64 + 3.0) * (counts[2] as f64 + 3.0);
		if !(cells.is_finite() && cells <= NARROWBAND_MAX_LATTICE_CELLS) {
			return None;
		}
		let nx = counts[0] as usize + 3;
		let ny = counts[1] as usize + 3;
		let nz = counts[2] as usize + 3;
		let origin = domain.min - Vec3::splat(vs);
		Some(Self { dims: [nx, ny, nz], origin, vs })
	}

	/// World position of lattice point `(i, j, k)`.
	#[inline]
	fn point(&self, i: usize, j: usize, k: usize) -> Vec3 {
		self.origin + Vec3::new(i as f32, j as f32, k as f32) * self.vs
	}

	/// Flat lattice-point index used as the SDF cache key.
	#[inline]
	fn point_index(&self, i: usize, j: usize, k: usize) -> usize {
		i + self.dims[0] * (j + self.dims[1] * k)
	}

	/// Cell counts per axis (`dims - 1`).
	#[inline]
	fn cell_dims(&self) -> [usize; 3] {
		[self.dims[0] - 1, self.dims[1] - 1, self.dims[2] - 1]
	}
}


/// Memoized corner-sample provider over a [`Lattice`].
struct CornerCache<'a, S: Sdf + ?Sized> {
	sdf: &'a S,
	lattice: &'a Lattice,
	cache: HashMap<usize, f32>,
	/// distinct SDF evaluations — the honest cost of the narrow band
	evals: usize,
}

impl<'a, S: Sdf + ?Sized> CornerCache<'a, S> {
	fn new(sdf: &'a S, lattice: &'a Lattice) -> Self {
		Self { sdf, lattice, cache: HashMap::new(), evals: 0 }
	}

	/// Signed distance at lattice point `(i, j, k)`, evaluating the SDF at most
	/// once per point.
	#[inline]
	fn sample(&mut self, i: usize, j: usize, k: usize) -> f32 {
		let key = self.lattice.point_index(i, j, k);
		match self.cache.entry(key) {
			Entry::Occupied(e) => *e.get(),
			Entry::Vacant(e) => {
				self.evals += 1;
				let v = self.sdf.distance(self.lattice.point(i, j, k));
				*e.insert(v)
			}
		}
	}

	/// Gather the eight corner samples of cell `(cx, cy, cz)` in Surface Nets
	/// corner order (`g = x | y<<1 | z<<2`) and the inside/outside mask.
	#[inline]
	fn cell(&mut self, cx: usize, cy: usize, cz: usize) -> ([f32; 8], u32) {
		let mut grid = [0f32; 8];
		let mut mask = 0u32;
		// `g` is a 3-bit cube-corner code (its bits ARE the offsets), not a position.
		#[allow(clippy::needless_range_loop)]
		for g in 0..8usize {
			let (oi, oj, ok) = (g & 1, (g >> 1) & 1, (g >> 2) & 1);
			let val = self.sample(cx + oi, cy + oj, cz + ok);
			grid[g] = val;
			if val < 0.0 {
				mask |= 1 << g;
			}
		}
		(grid, mask)
	}
}

/// A straddling cell carries a surface vertex; fully-inside / fully-outside
/// cells do not.
#[inline]
fn straddles(mask: u32) -> bool {
	mask != 0 && mask != 0xff
}

/// Linear cell index in the cell lattice.
#[inline]
fn cell_index(c: [usize; 3], cdims: [usize; 3]) -> usize {
	c[0] + cdims[0] * (c[1] + cdims[1] * c[2])
}

/// Discover every surface-straddling cell reachable from `seeds` via a
/// face-adjacent flood fill, returning them in deterministic discovery order.
/// `visited` accumulates *every* cell whose mask was evaluated (the work
/// measure) including non-straddling boundary cells touched by the frontier.
fn flood_fill<S: Sdf + ?Sized>(
	cache: &mut CornerCache<'_, S>,
	cdims: [usize; 3],
	seeds: &[[usize; 3]],
	visited: &mut usize,
) -> Vec<[usize; 3]> {
	let mut active: Vec<[usize; 3]> = Vec::new();
	let mut seen: HashSet<usize> = HashSet::new();
	let mut queue: VecDeque<[usize; 3]> = VecDeque::new();

	for &s in seeds {
		if s[0] < cdims[0] && s[1] < cdims[1] && s[2] < cdims[2] {
			let idx = cell_index(s, cdims);
			if seen.insert(idx) {
				queue.push_back(s);
			}
		}
	}

	// Face-adjacent (6-connected) neighbour offsets in the cell lattice.
	const STEPS: [(isize, isize, isize); 6] =
		[(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)];

	while let Some(c) = queue.pop_front() {
		let (_, mask) = cache.cell(c[0], c[1], c[2]);
		*visited += 1;
		if !straddles(mask) {
			// A non-straddling cell is a boundary of the band; do not expand it.
			continue;
		}
		active.push(c);
		for (dx, dy, dz) in STEPS {
			let nx = c[0] as isize + dx;
			let ny = c[1] as isize + dy;
			let nz = c[2] as isize + dz;
			if nx < 0 || ny < 0 || nz < 0 {
				continue;
			}
			let (nx, ny, nz) = (nx as usize, ny as usize, nz as usize);
			if nx >= cdims[0] || ny >= cdims[1] || nz >= cdims[2] {
				continue;
			}
			let nidx = cell_index([nx, ny, nz], cdims);
			if seen.insert(nidx) {
				queue.push_back([nx, ny, nz]);
			}
		}
	}

	active
}

/// Locate seed cells that straddle the surface.
///
/// A Lipschitz-safe coarse **block** scan: the lattice is tiled into blocks of
/// `B` cells; the distance at each block centre is sampled, and because the
/// fields here are (at worst) 1-Lipschitz — exact SDFs and the non-expansive
/// CSG `min`/`max`/offset/shell of them — a block whose centre distance exceeds
/// the block's half-diagonal provably contains no surface and is skipped;
/// otherwise every cell in the block is fine-scanned for straddling seeds.
///
/// Enumerate every surface-straddling cell by OCTREE descent: a sub-box whose
/// centre sample satisfies |d| > half-diagonal + one-cell margin cannot contain
/// a straddling cell (1-Lipschitz pruning) and is skipped whole; everything
/// else splits until cell level. Unlike the earlier first-seed-per-block scan,
/// no disconnected sheet can hide inside a coarse block (a gyroid trimmed by a
/// shell routinely puts several disjoint sheet fragments in one block — the
/// unseeded ones were silently dropped, leaving cracks), and unlike a full
/// scan the cost stays O(surface cells · log n). (For a heavily non-metric
/// field — a large uniform scale, or a raw `Gyroid` — redistance first to
/// restore the 1-Lipschitz property.) Every evaluated sample counts toward
/// `visited`.
/// How the narrow band finds the surface.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum SeedMode {
	/// Octree enumeration of EVERY surface-straddling cell — no disconnected
	/// sheet can be missed, whatever shares a block with what. ~2× the SDF
	/// evaluations of `Fast`; the default because silent cracks cost more.
	#[default]
	Exhaustive,
	/// One seed per surface-touching block + flood fill. Cheapest, and exact
	/// for any surface whose sheets are cell-connected — but two DISCONNECTED
	/// sheets sharing one coarse block can leave the second unmeshed (cracks).
	/// Use for interactive previews or fields known to be single-sheet.
	Fast,
}

fn find_seeds<S: Sdf + ?Sized>(
	cache: &mut CornerCache<'_, S>,
	cdims: [usize; 3],
	visited: &mut usize,
	mode: SeedMode,
) -> Vec<[usize; 3]> {
	let mut seeds: Vec<[usize; 3]> = Vec::new();
	if cdims.contains(&0) {
		return seeds;
	}
	let vs = cache.lattice.vs;
	if mode == SeedMode::Fast {
		// one seed per surface-touching 8³ block; the flood fill grows each
		// CONNECTED component from any one of its cells
		const B: usize = 8;
		let block_radius = (B as f32) * vs * 0.5 * 3.0_f32.sqrt();
		let mut bz = 0;
		while bz < cdims[2] {
			let mut by = 0;
			while by < cdims[1] {
				let mut bx = 0;
				while bx < cdims[0] {
					let ci = (bx + B / 2).min(cdims[0]);
					let cj = (by + B / 2).min(cdims[1]);
					let ck = (bz + B / 2).min(cdims[2]);
					let d = cache.sample(ci, cj, ck);
					*visited += 1;
					if d.abs() <= block_radius {
						let (xe, ye, ze) = ((bx + B).min(cdims[0]), (by + B).min(cdims[1]), (bz + B).min(cdims[2]));
						'block: for cz in bz..ze {
							for cy in by..ye {
								for cx in bx..xe {
									let (_, mask) = cache.cell(cx, cy, cz);
									*visited += 1;
									if straddles(mask) {
										seeds.push([cx, cy, cz]);
										break 'block;
									}
								}
							}
						}
					}
					bx += B;
				}
				by += B;
			}
			bz += B;
		}
		return seeds;
	}
	// iterative stack of cell-index boxes [x0, x1) × [y0, y1) × [z0, z1)
	let mut stack: Vec<[usize; 6]> = vec![[0, cdims[0], 0, cdims[1], 0, cdims[2]]];
	while let Some([x0, x1, y0, y1, z0, z1]) = stack.pop() {
		let (nx, ny, nz) = (x1 - x0, y1 - y0, z1 - z0);
		if nx == 0 || ny == 0 || nz == 0 {
			continue;
		}
		if nx <= 4 && ny <= 4 && nz <= 4 {
			// small enough: scan every cell — ALL straddling cells become seeds,
			// so no disconnected sheet inside the box can be missed
			for cz in z0..z1 {
				for cy in y0..y1 {
					for cx in x0..x1 {
						let (_, mask) = cache.cell(cx, cy, cz);
						*visited += 1;
						if straddles(mask) {
							seeds.push([cx, cy, cz]);
						}
					}
				}
			}
			continue;
		}
		// centre lattice point of the box; prune when the field is provably clear
		let ci = (x0 + nx / 2).min(cdims[0]);
		let cj = (y0 + ny / 2).min(cdims[1]);
		let ck = (z0 + nz / 2).min(cdims[2]);
		let d = cache.sample(ci, cj, ck);
		*visited += 1;
		// farthest cell CORNER of the box from the centre sample: half the box
		// diagonal, plus the ≤ half-cell-per-axis quantisation of the centre
		// lattice point (√3/2 of a cell)
		let half_diag = 0.5 * vs * (((nx * nx + ny * ny + nz * nz) as f32).sqrt() + 3.0_f32.sqrt());
		if d.abs() > half_diag {
			continue;
		}
		let (mx, my, mz) = (x0 + nx / 2, y0 + ny / 2, z0 + nz / 2);
		for &(a0, a1) in &[(x0, mx), (mx, x1)] {
			for &(b0, b1) in &[(y0, my), (my, y1)] {
				for &(c0, c1) in &[(z0, mz), (mz, z1)] {
					if a1 > a0 && b1 > b0 && c1 > c0 {
						stack.push([a0, a1, b0, b1, c0, c1]);
					}
				}
			}
		}
	}
	seeds
}

/// Surface-tracking Surface Nets. Produces a watertight mesh equivalent to the
/// dense [`crate::surface_nets`], but evaluates only cells in the narrow band
/// around the surface.
///
/// One cell of padding is added on every side (matching the dense mesher) so a
/// surface reaching the domain bounds still closes; winding is corrected to
/// outward (positive volume).
pub fn surface_nets_narrowband<S>(sdf: &S, domain: Aabb, resolution: impl Into<Resolution>) -> Mesh
where
	S: Sdf + ?Sized + Sync,
{
	mesh_and_visited(sdf, domain, resolution, Strategy::SurfaceNets, SeedMode::Exhaustive).0
}

/// Surface-tracking **Dual Contouring**: a watertight mesh equivalent to the
/// dense [`crate::dual_contour`] (sharp features preserved via per-cell QEF
/// vertices), but evaluating only cells in the narrow band around the surface,
/// so cost scales with surface area rather than volume.
pub fn dual_contour_narrowband<S>(sdf: &S, domain: Aabb, resolution: impl Into<Resolution>) -> Mesh
where
	S: Sdf + ?Sized + Sync,
{
	mesh_and_visited(sdf, domain, resolution, Strategy::DualContour, SeedMode::Exhaustive).0
}

/// Like [`dual_contour_narrowband`] but also returns the number of evaluated cells.
pub fn dual_contour_narrowband_with_visited<S>(
	sdf: &S,
	domain: Aabb,
	resolution: impl Into<Resolution>,
) -> (Mesh, usize)
where
	S: Sdf + ?Sized + Sync,
{
	mesh_and_visited(sdf, domain, resolution, Strategy::DualContour, SeedMode::Fast)
}

/// Like [`surface_nets_narrowband`] but also returns the number of cells whose
/// corner masks were evaluated (seeding + flood fill). For an area-scaling
/// surface this is far below `dims.x * dims.y * dims.z`.
pub fn surface_nets_narrowband_with_visited<S>(
	sdf: &S,
	domain: Aabb,
	resolution: impl Into<Resolution>,
) -> (Mesh, usize)
where
	S: Sdf + ?Sized + Sync,
{
	mesh_and_visited(sdf, domain, resolution, Strategy::SurfaceNets, SeedMode::Fast)
}

/// Core implementation: seed, flood-fill, then march the active cells.
fn mesh_and_visited<S>(sdf: &S, domain: Aabb, resolution: impl Into<Resolution>, strategy: Strategy, mode: SeedMode) -> (Mesh, usize)
where
	S: Sdf + ?Sized + Sync,
{
	let lattice = match Lattice::resolve(domain, resolution) {
		Some(l) => l,
		None => return (Mesh::new(), 0),
	};
	let cdims = lattice.cell_dims();
	if cdims[0] == 0 || cdims[1] == 0 || cdims[2] == 0 {
		return (Mesh::new(), 0);
	}

	let mut cache = CornerCache::new(sdf, &lattice);
	let mut visited = 0usize; // legacy per-probe counter, superseded by cache.evals

	let seeds = find_seeds(&mut cache, cdims, &mut visited, mode);
	if seeds.is_empty() {
		return (Mesh::new(), cache.evals);
	}
	let active = flood_fill(&mut cache, cdims, &seeds, &mut visited);
	let visited = cache.evals; // distinct SDF evaluations = the honest cost
	if active.is_empty() {
		return (Mesh::new(), visited);
	}

	let mesh = march_active(sdf, &lattice, cdims, &mut cache, &active, strategy);
	(mesh, visited)
}

/// March the active straddling cells: place one vertex per cell (Surface Nets
/// averaging) and emit a quad across each straddling minimal edge whose four
/// incident cells all carry a vertex.
fn march_active<S>(
	sdf: &S,
	lattice: &Lattice,
	cdims: [usize; 3],
	cache: &mut CornerCache<'_, S>,
	active: &[[usize; 3]],
	strategy: Strategy,
) -> Mesh
where
	S: Sdf + ?Sized,
{
	let (cube_edges, edge_table) = edge_tables();
	let cell_stride = [1usize, cdims[0], cdims[0] * cdims[1]];
	// World position of a cell's corner `c`.
	let corner_world = |cell: [usize; 3], c: usize| -> Vec3 {
		let o = CORNER_OFFSET[c];
		lattice.origin
			+ Vec3::new((cell[0] + o[0]) as f32, (cell[1] + o[1]) as f32, (cell[2] + o[2]) as f32) * lattice.vs
	};

	let mut mesh = Mesh::new();
	// Vertex id per active cell index. Sparse: only active cells appear.
	let mut cell_vertex: HashMap<usize, u32> = HashMap::with_capacity(active.len());

	// Pass 1: place one vertex per active cell so every cell has an id before
	// quad emission. Surface Nets averages edge crossings; Dual Contouring solves
	// a QEF from Hermite data (refined crossing + analytic gradient).
	for &c in active {
		let (grid, mask) = cache.cell(c[0], c[1], c[2]);
		debug_assert!(straddles(mask));
		let edge_mask = edge_table[mask as usize];

		let placed: Option<(Vec3, Vec3)> = match strategy {
			Strategy::SurfaceNets => {
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
					None
				} else {
					v /= e_count;
					let world = lattice.origin + (Vec3::new(c[0] as f32, c[1] as f32, c[2] as f32) + v) * lattice.vs;
					Some((world, sdf.gradient(world)))
				}
			}
			Strategy::DualContour => {
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
					let p = refine_crossing(sdf, corner_world(c, c0), corner_world(c, c1), g0, g1);
					planes.push((p, sdf.gradient(p)));
					centroid += p;
				}
				if planes.is_empty() {
					None
				} else {
					centroid /= planes.len() as f32;
					let cell_min = corner_world(c, 0);
					let vertex = solve_qef(&planes, centroid, cell_min, cell_min + Vec3::splat(lattice.vs));
					Some((vertex, sdf.gradient(vertex)))
				}
			}
		};

		let Some((world, normal)) = placed else { continue };
		let idx = cell_index(c, cdims);
		let vid = mesh.push_vertex(world);
		mesh.normals.push(normal);
		cell_vertex.insert(idx, vid);
	}

	// Pass 2: emit quads. For each active cell, each straddling minimal edge
	// (from corner 0, axes 0/1/2) joins the four cells around that edge.
	for &c in active {
		let (_, mask) = cache.cell(c[0], c[1], c[2]);
		let edge_mask = edge_table[mask as usize];
		let idx = cell_index(c, cdims);
		let q0 = match cell_vertex.get(&idx) {
			Some(&v) => v,
			None => continue,
		};
		for axis in 0..3usize {
			if edge_mask & (1 << axis) == 0 {
				continue;
			}
			let iu = (axis + 1) % 3;
			let iv = (axis + 2) % 3;
			let cu = c[iu];
			let cv = c[iv];
			if cu == 0 || cv == 0 {
				continue; // a neighbour cell would be out of range
			}
			let du = cell_stride[iu];
			let dv = cell_stride[iv];
			let q1 = match cell_vertex.get(&(idx - du)) {
				Some(&v) => v,
				None => continue,
			};
			let q2 = match cell_vertex.get(&(idx - du - dv)) {
				Some(&v) => v,
				None => continue,
			};
			let q3 = match cell_vertex.get(&(idx - dv)) {
				Some(&v) => v,
				None => continue,
			};
			let (a, b, c2, d) = if mask & 1 != 0 { (q0, q1, q2, q3) } else { (q0, q3, q2, q1) };
			mesh.push_triangle(a, b, c2);
			mesh.push_triangle(a, c2, d);
		}
	}

	mesh.ensure_outward();
	mesh
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::ops::Node;
	use crate::primitives::{Cuboid, Sphere};
	use crate::{dual_contour, surface_nets};
	use kernel_core::math::Aabb;

	/// Cell count of the padded lattice the meshers share, for the area-scaling
	/// comparison.
	fn dense_cell_count(domain: Aabb, vs: f32) -> usize {
		let l = Lattice::resolve(domain, Resolution::VoxelSize(vs)).unwrap();
		let c = l.cell_dims();
		c[0] * c[1] * c[2]
	}

	#[test]
	fn sphere_matches_dense_and_is_watertight() {
		let sphere = Node::primitive(Sphere::new(Vec3::ZERO, 10.0));
		let domain = sphere.bounds();
		// Finer resolution makes the area-vs-volume scaling clear (the surface
		// band grows as 1/vs² while the dense cell count grows as 1/vs³).
		let vs = 0.3f32;

		let dense = surface_nets(&sphere, domain, Resolution::VoxelSize(vs));
		let (nb, visited) = surface_nets_narrowband_with_visited(&sphere, domain, Resolution::VoxelSize(vs));

		let dense_v = dense.signed_volume();
		let nb_v = nb.signed_volume();
		let total_cells = dense_cell_count(domain, vs);

		assert!(nb.is_watertight(), "narrow-band sphere must be watertight");
		assert!(
			(nb_v - dense_v).abs() / dense_v.abs() < 1e-3,
			"narrow-band volume {nb_v} should match dense {dense_v}"
		);
		// Surface-area scaling. `visited` now counts DISTINCT SDF evaluations
		// (the honest cost — the old per-probe counter undercounted by reusing
		// cached corners for free): the band evaluates well under half of what
		// the dense mesher must.
		assert!(
			visited * 2 < total_cells,
			"visited {visited} (true SDF evals) should be well below dense cell count {total_cells}"
		);
	}

	#[test]
	fn thin_shell_matches_dense_and_is_watertight() {
		// A thin spherical shell via the shell op: two close, concentric surfaces.
		let shell = Node::primitive(Sphere::new(Vec3::ZERO, 10.0)).shell(0.8);
		let domain = shell.bounds();
		let vs = 0.5f32;

		let dense = surface_nets(&shell, domain, Resolution::VoxelSize(vs));
		let (nb, visited) = surface_nets_narrowband_with_visited(&shell, domain, Resolution::VoxelSize(vs));

		let dense_v = dense.signed_volume();
		let nb_v = nb.signed_volume();
		let total_cells = dense_cell_count(domain, vs);

		assert!(nb.is_watertight(), "narrow-band shell must be watertight");
		assert!(!dense.is_empty() && dense_v.abs() > 0.0, "dense shell should be non-empty");
		assert!(
			(nb_v - dense_v).abs() / dense_v.abs() < 5e-3,
			"narrow-band shell volume {nb_v} should match dense {dense_v}"
		);
		// a THIN shell in a tight domain is nearly all band — the honest bar is
		// simply beating dense, not a fixed multiple
		assert!(
			visited < total_cells,
			"visited {visited} (true SDF evals) should beat the dense cell count {total_cells}"
		);
	}

	#[test]
	fn two_components_both_meshed() {
		// Two disjoint spheres: seeding from several coarse-scan hits must capture
		// both connected components.
		let a = Node::primitive(Sphere::new(Vec3::new(-15.0, 0.0, 0.0), 6.0));
		let b = Node::primitive(Sphere::new(Vec3::new(15.0, 0.0, 0.0), 6.0));
		let both = a.union(b);
		let domain = both.bounds();
		let vs = 0.5f32;

		let dense = surface_nets(&both, domain, Resolution::VoxelSize(vs));
		let nb = surface_nets_narrowband(&both, domain, Resolution::VoxelSize(vs));

		let dense_v = dense.signed_volume();
		let nb_v = nb.signed_volume();
		assert!(nb.is_watertight(), "two-component narrow-band mesh must be watertight");
		assert!(
			(nb_v - dense_v).abs() / dense_v.abs() < 1e-2,
			"two-component volume {nb_v} should match dense {dense_v}"
		);
	}

	#[test]
	fn many_small_components_match_dense() {
		// Adversarial: small spheres spaced far apart — each is tiny relative to
		// any coarse block, exactly the case a stride scan would step over. The
		// block scan must seed every one. Watertightness alone is NOT a valid
		// oracle here (a partial union of closed spheres is still watertight), so
		// assert both triangle count and volume against the dense mesher.
		let mut part: Option<Node> = None;
		for gx in -1..=1 {
			for gy in -1..=1 {
				let s = Node::primitive(Sphere::new(
					Vec3::new(gx as f32 * 18.0, gy as f32 * 18.0, 0.0),
					1.5,
				));
				part = Some(match part {
					None => s,
					Some(p) => p.union(s),
				});
			}
		}
		let part = part.unwrap();
		let domain = part.bounds().pad(1.0);
		let vs = 0.4f32;

		let dense = surface_nets(&part, domain, Resolution::VoxelSize(vs));
		let nb = surface_nets_narrowband(&part, domain, Resolution::VoxelSize(vs));

		assert!(nb.is_watertight(), "narrow-band must be watertight");
		let dv = dense.signed_volume();
		let nv = nb.signed_volume();
		assert!(
			(nv - dv).abs() / dv.abs() < 1e-2,
			"narrow-band volume {nv} should match dense {dv} (no dropped components)"
		);
		// Structural completeness: triangle counts must match closely, not merely
		// produce *a* watertight mesh.
		let dt = dense.triangle_count() as i64;
		let nt = nb.triangle_count() as i64;
		assert!(
			(dt - nt).abs() <= dt / 50,
			"narrow-band tri count {nt} should match dense {dt} (all 9 components present)"
		);
	}

	#[test]
	fn dual_contour_narrowband_matches_dense_and_is_sharp() {
		// A cube: narrow-band Dual Contouring must equal dense DC (volume,
		// watertight), preserve the sharp corners, and visit far fewer cells.
		let cube = Node::primitive(Cuboid::new(Vec3::ZERO, Vec3::splat(8.0)));
		let domain = cube.bounds().pad(2.0);
		let vs = 0.4f32;

		let dense = dual_contour(&cube, domain, Resolution::VoxelSize(vs));
		let (nb, visited) = dual_contour_narrowband_with_visited(&cube, domain, Resolution::VoxelSize(vs));

		assert!(nb.is_watertight(), "narrow-band DC must be watertight");
		let (dv, nv) = (dense.signed_volume(), nb.signed_volume());
		assert!((nv - dv).abs() / dv.abs() < 1e-2, "narrow-band DC vol {nv} vs dense {dv}");

		// Sharp feature preserved: a vertex sits right at each true cube corner.
		let nearest = |m: &kernel_core::Mesh, c: Vec3| {
			m.positions.iter().map(|&p| (p - c).length()).fold(f32::INFINITY, f32::min)
		};
		for corner in [Vec3::splat(8.0), Vec3::new(-8.0, 8.0, -8.0), Vec3::splat(-8.0)] {
			assert!(nearest(&nb, corner) < 0.4 * vs, "sharp corner {corner:?} preserved");
		}
		// Area scaling: only a fraction of the volume is evaluated.
		assert!(visited < dense_cell_count(domain, vs), "narrow band visits fewer cells than dense");
	}

	#[test]
	fn degenerate_domain_is_empty() {
		let sphere = Node::primitive(Sphere::new(Vec3::ZERO, 5.0));
		let bad = Aabb::new(Vec3::splat(1.0), Vec3::splat(-1.0)); // inverted
		let nb = surface_nets_narrowband(&sphere, bad, Resolution::VoxelSize(0.5));
		assert!(nb.is_empty(), "inverted domain must yield an empty mesh");
	}
}

#[cfg(test)]
mod seed_mode_tests {
	use super::*;
	use crate::{Cone, Node, Tpms, TpmsKind};
	use kernel_core::math::{Aabb, Vec3};

	/// The repro that motivated SeedMode: a gyroid trimmed by a conical shell
	/// puts several DISCONNECTED sheet fragments inside one seeding block.
	/// Fast's first-seed-per-block DROPPED whole sheets (true boundary cracks,
	/// 1-use edges); Exhaustive must lose nothing: no 1-use edge anywhere.
	/// (4-use PINCH edges are a separate, known property of single-vertex
	/// cells on near-touching sheets — `make_manifold` resolves those; a fully
	/// pinch-free narrowband is tracked as open work in manifold_dc's docs.)
	#[test]
	fn gyroid_shell_multisheet_loses_no_sheet_under_exhaustive() {
		let outer = |grow: f32| {
			Node::primitive(Cone::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 40.0), 18.0 + grow, 26.0 + grow))
		};
		let shell = outer(0.0).difference(outer(-6.0));
		let region = Aabb::new(Vec3::new(-30.0, -30.0, -2.0), Vec3::new(30.0, 30.0, 42.0));
		let gy = Node::primitive(Tpms::network(region, TpmsKind::Gyroid, 8.0, 0.0));
		let lattice = gy.intersection(shell);
		let domain = Aabb::new(Vec3::new(-30.0, -30.0, -2.0), Vec3::new(30.0, 30.0, 44.0));
		let m = surface_nets_narrowband(&lattice, domain, Resolution::VoxelSize(0.5));
		let edge_uses = |m: &kernel_core::Mesh| {
			let mut edges = std::collections::HashMap::new();
			for t in m.indices.chunks_exact(3) {
				for k in 0..3 {
					let (a, b) = (t[k], t[(k + 1) % 3]);
					*edges.entry(if a < b { (a, b) } else { (b, a) }).or_insert(0u32) += 1;
				}
			}
			edges
		};
		let boundary = edge_uses(&m).values().filter(|&&c| c == 1).count();
		// and the full supported pipeline must be watertight end to end
		let clean = kernel_core::make_manifold(&crate::manifold_dual_contour(&lattice, domain, Resolution::VoxelSize(0.5)));
		let clean_bad = edge_uses(&clean).values().filter(|&&c| c != 2).count();
		assert!(
			boundary == 0 && m.indices.len() > 3000 && clean.is_watertight() && clean_bad == 0,
			"multi-sheet lattice: exhaustive band must lose no sheet (boundary edges {boundary}, tris {}); manifold pipeline must be watertight (bad edges {clean_bad}, watertight {})",
			m.indices.len() / 3,
			clean.is_watertight()
		);
	}
}
