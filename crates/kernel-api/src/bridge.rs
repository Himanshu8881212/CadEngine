//! The ACE bridge — density-grid interchange with voxel physics pipelines.
//!
//! ACE (Autonomous Computational Engineer) runs topology optimization and
//! hex8 voxel FEA over two numpy arrays: `solid_fraction.npy` (float32 in
//! `[0,1]`, C-order, shape `(nx, ny, nz)` indexed `rho[i, j, k]` with
//! `i↔x, j↔y, k↔z`, voxel CENTERS at `origin + (index + 0.5)·h`, mm) and a
//! string `region_kind.npy` it derives itself. These two ops make LMCAD the
//! geometry authority on both ends of that loop:
//!
//! - [`sample_density_npy`]: any bound SOLID (via the winding-number
//!   `MeshSdf` bridge) or implicit tree → `solid_fraction.npy` in exactly
//!   that contract (supersampled fractions, not just center occupancy).
//! - [`mesh_density_npy`]: an optimized density `.npy` → level-set →
//!   redistanced [`VoxelGrid`] → watertight narrow-band mesh → STL, with
//!   the same report fields ACE's `render.emit_stl` contract promises
//!   (`ok, volume_mm3, num_triangles, watertight`), but produced by the
//!   kernel's gated meshing pipeline instead of raw marching cubes.
//!
//! The `.npy` reader/writer below covers the exact dialect numpy emits for
//! these arrays (v1.0 header, little-endian `<f4`/`<f8`, C-order) — nothing
//! more, loudly rejecting anything else.

use kernel_core::math::Vec3;
use kernel_core::Mesh;
use kernel_implicit::{MeshSdf, Sdf, VoxelGrid};

/// Parse a `.npy` v1.x header + payload into `(shape, f32 data)` (C-order).
/// Accepts `<f4` and `<f8` (f8 is narrowed), rejects fortran order and any
/// other dtype with a descriptive error.
pub fn read_npy_f32(bytes: &[u8]) -> Result<(Vec<usize>, Vec<f32>), String> {
	if bytes.len() < 10 || &bytes[..6] != b"\x93NUMPY" {
		return Err("not a .npy file (bad magic)".into());
	}
	let (hlen, hoff) = match bytes[6] {
		1 => (u16::from_le_bytes([bytes[8], bytes[9]]) as usize, 10usize),
		2 | 3 => {
			if bytes.len() < 12 {
				return Err("truncated .npy header".into());
			}
			(u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]) as usize, 12usize)
		}
		v => return Err(format!("unsupported .npy major version {v}")),
	};
	let header = std::str::from_utf8(bytes.get(hoff..hoff + hlen).ok_or("truncated .npy header")?)
		.map_err(|_| "non-utf8 .npy header".to_string())?;
	let descr = if header.contains("'<f4'") || header.contains("\"<f4\"") {
		4
	} else if header.contains("'<f8'") || header.contains("\"<f8\"") {
		8
	} else {
		return Err(format!("unsupported dtype in .npy header (need little-endian float32/float64): {header}"));
	};
	if header.contains("'fortran_order': True") {
		return Err("fortran-order .npy is not supported (the ACE contract is C-order)".into());
	}
	let spos = header.find("'shape':").or_else(|| header.find("\"shape\":")).ok_or("no shape in .npy header")?;
	let open = header[spos..].find('(').ok_or("malformed shape")? + spos;
	let close = header[open..].find(')').ok_or("malformed shape")? + open;
	let shape: Vec<usize> = header[open + 1..close]
		.split(',')
		.filter(|t| !t.trim().is_empty())
		.map(|t| t.trim().parse::<usize>().map_err(|_| format!("bad shape token '{t}'")))
		.collect::<Result<_, _>>()?;
	let count: usize = shape.iter().product();
	let data = &bytes[hoff + hlen..];
	if data.len() < count * descr {
		return Err(format!("payload too short: {} bytes for {count} × {descr}", data.len()));
	}
	let mut out = Vec::with_capacity(count);
	if descr == 4 {
		for c in data[..count * 4].chunks_exact(4) {
			out.push(f32::from_le_bytes([c[0], c[1], c[2], c[3]]));
		}
	} else {
		for c in data[..count * 8].chunks_exact(8) {
			out.push(f64::from_le_bytes([c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7]]) as f32);
		}
	}
	Ok((shape, out))
}

/// Serialize a C-order float32 array as `.npy` v1.0 — the exact dialect
/// `np.load` expects for ACE's `solid_fraction.npy`.
pub fn write_npy_f32(shape: &[usize], data: &[f32]) -> Vec<u8> {
	let shape_txt = match shape.len() {
		1 => format!("({},)", shape[0]),
		_ => format!("({})", shape.iter().map(|s| s.to_string()).collect::<Vec<_>>().join(", ")),
	};
	let mut header = format!("{{'descr': '<f4', 'fortran_order': False, 'shape': {shape_txt}, }}");
	let unpadded = 10 + header.len() + 1;
	let pad = (64 - unpadded % 64) % 64;
	header.push_str(&" ".repeat(pad));
	header.push('\n');
	let mut out = Vec::with_capacity(10 + header.len() + data.len() * 4);
	out.extend_from_slice(b"\x93NUMPY\x01\x00");
	out.extend_from_slice(&(header.len() as u16).to_le_bytes());
	out.extend_from_slice(header.as_bytes());
	for v in data {
		out.extend_from_slice(&v.to_le_bytes());
	}
	out
}

/// Sample any [`Sdf`] into the ACE density contract: shape `(nx, ny, nz)`,
/// C-order `rho[i, j, k]`, voxel centers `origin + (idx + 0.5)·h`, each value
/// the inside-fraction of `supersample³` stratified sub-points (so a boundary
/// voxel reads a genuine fraction, which SIMP loops want, rather than 0/1).
pub fn sample_density<S: Sdf + ?Sized>(sdf: &S, origin: Vec3, h: f32, shape: [usize; 3], supersample: usize) -> Vec<f32> {
	let (nx, ny, nz) = (shape[0], shape[1], shape[2]);
	let ss = supersample.max(1);
	let inv = 1.0 / ss as f32;
	let n_sub = (ss * ss * ss) as f32;
	let mut out = Vec::with_capacity(nx * ny * nz);
	for i in 0..nx {
		for j in 0..ny {
			for k in 0..nz {
				let base = origin + Vec3::new(i as f32, j as f32, k as f32) * h;
				let mut hits = 0u32;
				for a in 0..ss {
					for b in 0..ss {
						for c in 0..ss {
							let p = base + Vec3::new((a as f32 + 0.5) * inv, (b as f32 + 0.5) * inv, (c as f32 + 0.5) * inv) * h;
							if sdf.distance(p) < 0.0 {
								hits += 1;
							}
						}
					}
				}
				out.push(hits as f32 / n_sub);
			}
		}
	}
	out
}

/// Wrap a tessellated solid mesh as a winding-number SDF for sampling.
pub fn mesh_sdf(mesh: &Mesh) -> MeshSdf {
	MeshSdf::new(mesh)
}

/// Build a signed level-set [`VoxelGrid`] from a density array at threshold
/// `iso` (the field is `(iso − rho)·h`, negative inside), then redistance it
/// so downstream meshing sees a true distance field. Grid NODES sit at the
/// voxel centers (`origin + 0.5h`), matching the sampling convention.
pub fn density_to_grid(shape: [usize; 3], rho: &[f32], origin: Vec3, h: f32, iso: f32) -> VoxelGrid {
	let (nx, ny, nz) = (shape[0], shape[1], shape[2]);
	// grid lattice points at the voxel centers; note the flattening bases
	// differ: ACE is C-order (z fastest), VoxelGrid is x-fastest
	let node_origin = origin + Vec3::splat(0.5 * h);
	let mut data = vec![0.0f32; nx * ny * nz];
	for i in 0..nx {
		for j in 0..ny {
			for k in 0..nz {
				let rho_v = rho[(i * ny + j) * nz + k];
				data[i + nx * (j + ny * k)] = (iso - rho_v) * h;
			}
		}
	}
	let grid = VoxelGrid { origin: node_origin, voxel_size: h, dims: [nx, ny, nz], data };
	kernel_implicit::redistance(&grid)
}
