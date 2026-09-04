// Copyright (c) LMCAD. Licensed under the MIT License.

//! Redistancing (reinitialization) of a [`VoxelGrid`] via the **Fast Sweeping
//! Method** (FSM).
//!
//! CSG booleans (`min`/`max`) and smooth blends do not preserve a true
//! Euclidean signed distance away from the surface: `min(a, b)` is metric only
//! near the surface it actually bounds, and `smin` is deliberately
//! non-metric everywhere. Any operation that reads the field *off* the surface
//! — offsetting, shelling, sphere-tracing step bounds — is therefore wrong on
//! such a field.
//!
//! [`redistance`] fixes this. It keeps the **sign** of the input field at every
//! lattice point (which fixes the inside/outside topology and the zero level
//! set) and recomputes the **magnitude** as the viscosity solution of the
//! eikonal equation `|∇d| = 1`. The solver is Zhao's Fast Sweeping Method:
//! Godunov upwind updates applied in the eight (`2³`) diagonal Gauss-Seidel
//! sweep orderings; a small fixed number of full sweep-sets converges the band
//! that matters for offset/shell.

use crate::grid::VoxelGrid;
use kernel_core::sdf::Sdf;

/// Default number of full sweep-sets (each = all 8 directional sweeps). Two to
/// three sweep-sets resolve the field far past any practical offset distance;
/// three is a safe default that still costs only `O(N)` per set.
const DEFAULT_SWEEP_SETS: usize = 3;

/// Recompute a true unsigned Euclidean distance field from `grid` while keeping
/// the input field's sign at every lattice point, using the Fast Sweeping
/// Method to solve `|∇d| = 1`.
///
/// The returned grid shares `origin`, `voxel_size`, and `dims` with the input;
/// only `data` is replaced. Where the input has a genuine signed distance near
/// the surface the result is (to interpolation order) identical; away from the
/// surface — and after any boolean / smooth blend — the result is the corrected
/// metric distance, so offset and shell behave as expected.
///
/// Degenerate inputs (a field with no sign change, or a lattice too small to
/// sweep) are handled gracefully: a field with no zero crossing keeps its sign
/// and is pushed to the band edge rather than left at `+inf`.
pub fn redistance(grid: &VoxelGrid) -> VoxelGrid {
	redistance_with(grid, DEFAULT_SWEEP_SETS)
}

/// [`redistance`] with an explicit number of full sweep-sets (see
/// [`DEFAULT_SWEEP_SETS`]). Useful when a larger trusted band is required.
pub fn redistance_with(grid: &VoxelGrid, sweep_sets: usize) -> VoxelGrid {
	let [nx, ny, nz] = grid.dims;
	let n = nx * ny * nz;
	let h = grid.voxel_size.max(1e-6);

	// Preserve the sign of the input field at every lattice point. Zeros are
	// treated as "outside" (+1) so the convention is total and stable.
	let mut sign = vec![1.0f32; n];
	for (s, &d) in sign.iter_mut().zip(grid.data.iter()) {
		if d < 0.0 {
			*s = -1.0;
		}
	}

	// `dist` holds the running *unsigned* distance estimate (always >= 0).
	// A large but finite sentinel (not f32::INFINITY) keeps the Godunov update
	// arithmetic well-behaved while still acting as "unknown".
	let far = far_value(grid, h);
	let mut dist = vec![far; n];
	// `frozen` marks the seed cells adjacent to a sign change: their sub-voxel
	// distance is a Dirichlet boundary condition and must never be overwritten.
	let mut frozen = vec![false; n];

	seed_interface(grid, h, &sign, &mut dist, &mut frozen);

	// If there is no interface at all the field is single-signed; there is
	// nothing to sweep toward, so just clamp the sentinel and reapply the sign.
	if !frozen.iter().any(|&f| f) {
		return rebuild(grid, &dist, &sign, far);
	}

	for _ in 0..sweep_sets.max(1) {
		sweep_all_directions(&mut dist, &frozen, [nx, ny, nz], h);
	}

	rebuild(grid, &dist, &sign, far)
}

/// Convenience: sample `sdf` onto a fresh dense lattice covering `domain` at
/// voxel size `vs`, then redistance it. Equivalent to
/// `redistance(&VoxelGrid::from_sdf(sdf, domain, vs))`, which is the common path
/// after building a CSG tree.
pub fn redistanced<S>(sdf: &S, domain: kernel_core::math::Aabb, vs: f32) -> VoxelGrid
where
	S: Sdf + ?Sized + Sync,
{
	let grid = VoxelGrid::from_sdf(sdf, domain, vs);
	redistance(&grid)
}

/// The "unknown / far away" sentinel: larger than any reachable distance on
/// this lattice, yet finite so upwind differencing stays numerically clean.
#[inline]
fn far_value(grid: &VoxelGrid, h: f32) -> f32 {
	let [nx, ny, nz] = grid.dims;
	// Diagonal of the lattice is an upper bound on any true distance; pad it.
	let diag = ((nx + ny + nz) as f32) * h;
	diag.max(h) * 2.0
}

/// Linear index into the flat lattice (matching [`VoxelGrid`]'s own layout).
#[inline]
fn index(i: usize, j: usize, k: usize, dims: [usize; 3]) -> usize {
	i + dims[0] * (j + dims[1] * k)
}

/// Initialize the unsigned distance for every cell that touches the zero level
/// set. For an edge between a cell and a neighbour of opposite sign, the zero
/// crossing sits a fraction `t = |φ_c| / |φ_c − φ_n|` of a voxel away, so the
/// true distance from the cell to the surface along that edge is `t·h`. We keep
/// the smallest such estimate over all six neighbours and freeze the cell.
fn seed_interface(grid: &VoxelGrid, h: f32, sign: &[f32], dist: &mut [f32], frozen: &mut [bool]) {
	let dims = grid.dims;
	let [nx, ny, nz] = dims;
	let phi = &grid.data;

	for k in 0..nz {
		for j in 0..ny {
			for i in 0..nx {
				let c = index(i, j, k, dims);
				let sc = sign[c];
				let pc = phi[c].abs();
				let mut best = f32::INFINITY;

				// Each axis: inspect the lower and upper neighbour.
				let mut consider = |nidx: usize| {
					if sign[nidx] != sc {
						let pn = phi[nidx].abs();
						let denom = pc + pn;
						// `t` is the fractional position of the crossing measured
						// from the current cell toward the neighbour.
						let t = if denom > 1e-12 { pc / denom } else { 0.0 };
						let d = t * h;
						if d < best {
							best = d;
						}
					}
				};

				if i > 0 {
					consider(index(i - 1, j, k, dims));
				}
				if i + 1 < nx {
					consider(index(i + 1, j, k, dims));
				}
				if j > 0 {
					consider(index(i, j - 1, k, dims));
				}
				if j + 1 < ny {
					consider(index(i, j + 1, k, dims));
				}
				if k > 0 {
					consider(index(i, j, k - 1, dims));
				}
				if k + 1 < nz {
					consider(index(i, j, k + 1, dims));
				}

				if best.is_finite() {
					dist[c] = best;
					frozen[c] = true;
				}
			}
		}
	}
}

/// Run all eight diagonal Gauss-Seidel sweeps over the lattice once.
fn sweep_all_directions(dist: &mut [f32], frozen: &[bool], dims: [usize; 3], h: f32) {
	// The 8 orderings: each axis independently ascending (false) or descending
	// (true). Together they propagate causal information from every direction.
	for &dx in &[false, true] {
		for &dy in &[false, true] {
			for &dz in &[false, true] {
				sweep(dist, frozen, dims, h, dx, dy, dz);
			}
		}
	}
}

/// One directional Gauss-Seidel sweep applying the Godunov upwind eikonal
/// update at every non-frozen cell, visiting axes in the chosen orientations.
fn sweep(dist: &mut [f32], frozen: &[bool], dims: [usize; 3], h: f32, desc_x: bool, desc_y: bool, desc_z: bool) {
	let [nx, ny, nz] = dims;
	for kk in 0..nz {
		let k = if desc_z { nz - 1 - kk } else { kk };
		for jj in 0..ny {
			let j = if desc_y { ny - 1 - jj } else { jj };
			for ii in 0..nx {
				let i = if desc_x { nx - 1 - ii } else { ii };
				let c = index(i, j, k, dims);
				if frozen[c] {
					continue;
				}

				// Upwind (smaller) neighbour distance along each axis.
				let ax = upwind(dist, dims, i, nx, 0, j, k);
				let ay = upwind(dist, dims, j, ny, 1, i, k);
				let az = upwind(dist, dims, k, nz, 2, i, j);

				let candidate = godunov_solve(ax, ay, az, h);
				if candidate < dist[c] {
					dist[c] = candidate;
				}
			}
		}
	}
}

/// Minimum of the two neighbour distances along one axis (the upwind value the
/// Godunov scheme uses). `pos` is the index along `axis` (0=x, 1=y, 2=z); the
/// other two coordinates are passed in `a` and `b` in ascending-axis order.
#[inline]
fn upwind(dist: &[f32], dims: [usize; 3], pos: usize, len: usize, axis: usize, a: usize, b: usize) -> f32 {
	let at = |p: usize| -> f32 {
		let idx = match axis {
			0 => index(p, a, b, dims),
			1 => index(a, p, b, dims),
			_ => index(a, b, p, dims),
		};
		dist[idx]
	};
	let lo = if pos > 0 { at(pos - 1) } else { f32::INFINITY };
	let hi = if pos + 1 < len { at(pos + 1) } else { f32::INFINITY };
	lo.min(hi)
}

/// Godunov solution of `|∇d| = 1` at a cell given the upwind neighbour values
/// `a1 <= a2 <= a3` (after sorting) along the three axes, with uniform spacing
/// `h`. Solves the largest sub-system whose root stays below the next value:
///
/// - 1D: `d = a1 + h`
/// - 2D: `(d−a1)² + (d−a2)² = h²`
/// - 3D: `(d−a1)² + (d−a2)² + (d−a3)² = h²`
///
/// taking the larger quadratic root each time. Infinite (unknown) neighbours
/// drop out naturally.
fn godunov_solve(ax: f32, ay: f32, az: f32, h: f32) -> f32 {
	// Sort the (possibly infinite) candidates ascending; only the finite,
	// "small enough" ones enter each successive sub-system.
	let mut a = [ax, ay, az];
	a.sort_by(|p, q| p.partial_cmp(q).unwrap_or(std::cmp::Ordering::Equal));
	let [a1, a2, a3] = a;

	if !a1.is_finite() {
		// No known neighbour at all (should not happen once seeded).
		return f32::INFINITY;
	}

	// One-axis update.
	let mut d = a1 + h;
	if a2.is_finite() && d > a2 {
		// Two-axis quadratic: 2d² − 2(a1+a2)d + (a1²+a2²−h²) = 0.
		let sum = a1 + a2;
		let disc = 2.0 * h * h - (a1 - a2) * (a1 - a2);
		if disc >= 0.0 {
			d = 0.5 * (sum + disc.sqrt());
		}
		if a3.is_finite() && d > a3 {
			// Three-axis quadratic.
			let sum3 = a1 + a2 + a3;
			let sq = a1 * a1 + a2 * a2 + a3 * a3;
			let disc3 = sum3 * sum3 - 3.0 * (sq - h * h);
			if disc3 >= 0.0 {
				d = (sum3 + disc3.sqrt()) / 3.0;
			}
		}
	}
	d
}

/// Assemble the output grid: clamp the swept unsigned distances (replace any
/// remaining sentinel with the band edge) and reapply the stored sign.
fn rebuild(grid: &VoxelGrid, dist: &[f32], sign: &[f32], far: f32) -> VoxelGrid {
	let mut data = vec![0.0f32; dist.len()];
	for idx in 0..dist.len() {
		let mag = if dist[idx].is_finite() { dist[idx].min(far) } else { far };
		data[idx] = sign[idx] * mag;
	}
	VoxelGrid { origin: grid.origin, voxel_size: grid.voxel_size, dims: grid.dims, data }
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::primitives::Sphere;
	use kernel_core::math::{Aabb, Vec3};

	/// A deliberately non-metric field: a true sphere SDF scaled by `k`. Its
	/// zero set is unchanged (same sphere) but `|∇φ| = k ≠ 1`, so offsetting it
	/// would be wrong until redistanced.
	struct ScaledSphere {
		sphere: Sphere,
		k: f32,
	}

	impl Sdf for ScaledSphere {
		fn distance(&self, p: Vec3) -> f32 {
			self.sphere.distance(p) * self.k
		}
		fn bounds(&self) -> Aabb {
			self.sphere.bounds()
		}
	}

	#[test]
	fn redistanced_scaled_sphere_is_metric_and_sign_preserving() {
		let radius = 5.0f32;
		let center = Vec3::ZERO;
		let field = ScaledSphere { sphere: Sphere::new(center, radius), k: 3.0 };
		let domain = Aabb::from_center_half_extent(center, Vec3::splat(10.0));
		let vs = 0.25f32;

		let result = redistanced(&field, domain, vs);

		// Sample points at various true distances from the sphere surface; the
		// closed-form signed distance is `|p| − radius`.
		let probes = [
			Vec3::new(0.0, 0.0, 0.0),   // deep inside  → -5
			Vec3::new(2.0, 0.0, 0.0),   // inside       → -3
			Vec3::new(4.5, 0.0, 0.0),   // just inside  → -0.5
			Vec3::new(6.0, 0.0, 0.0),   // just outside → +1
			Vec3::new(8.0, 0.0, 0.0),   // outside      → +3
			Vec3::new(4.0, 4.0, 2.0),   // off-axis (len 6) → +1
			Vec3::new(-3.0, 1.0, -2.0), // off-axis inside  → -1.258...
		];

		let tol = 1.5 * vs; // within ~1 voxel of truth.
		for &p in &probes {
			let truth = p.length() - radius;
			let got = result.distance(p);
			assert!((got - truth).abs() <= tol, "redistanced value {got} vs truth {truth} at {p:?} (tol {tol})");
			// Sign must match the original field (which equals sign of truth).
			assert_eq!(got < 0.0, truth < 0.0, "sign flipped at {p:?}: got {got}, truth {truth}");
		}

		// Gradient magnitude must be ~1 after redistancing (it was ~3 before).
		// Use the grid's central-difference gradient (already normalized), so
		// instead verify the field slope directly via finite differences.
		let g = 0.5;
		for &p in &[Vec3::new(6.5, 0.0, 0.0), Vec3::new(0.0, 7.0, 0.0)] {
			let dx = result.distance(p + Vec3::new(g, 0.0, 0.0)) - result.distance(p - Vec3::new(g, 0.0, 0.0));
			let dy = result.distance(p + Vec3::new(0.0, g, 0.0)) - result.distance(p - Vec3::new(0.0, g, 0.0));
			let dz = result.distance(p + Vec3::new(0.0, 0.0, g)) - result.distance(p - Vec3::new(0.0, 0.0, g));
			let grad_mag = Vec3::new(dx, dy, dz).length() / (2.0 * g);
			assert!((grad_mag - 1.0).abs() <= 0.2, "gradient magnitude {grad_mag} not ~1 at {p:?}");
		}
	}

	#[test]
	fn single_signed_field_keeps_sign() {
		// A field with no zero crossing in the domain: a tiny sphere far away
		// so the whole sampled box is "outside" (+). Redistancing must not flip
		// signs or panic; it should remain entirely positive.
		let field = Sphere::new(Vec3::splat(100.0), 1.0);
		let domain = Aabb::from_center_half_extent(Vec3::ZERO, Vec3::splat(4.0));
		let grid = VoxelGrid::from_sdf(&field, domain, 0.5);
		let result = redistance(&grid);
		assert!(result.data.iter().all(|&d| d > 0.0), "single-signed field should stay positive");
	}
}
