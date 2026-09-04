// Copyright (c) LMCAD. Licensed under the MIT License.

//! [`GridField`] — a regular-grid sampled scalar field with trilinear
//! interpolation: the bridge that lets **simulation data drive geometry**.
//!
//! This is the generic carrier for ANY nodal/voxel scalar field — von Mises
//! stress, a graded density, temperature, a manufacturability score — loaded
//! from a NumPy `.npy` file (the format `tools/ace_fea_runner.py` writes as
//! `stress_field.npy` and `tools/stress_to_density.py` re-maps) or built
//! in-memory via [`GridField::from_data`]. It plugs straight into the
//! kernel's EXISTING grading mechanism: [`GridField::into_scalar_field`] /
//! [`GridField::into_grade_law`] produce the [`ScalarField`] closures consumed
//! by [`Node::offset_by`](crate::ops::Node::offset_by) (graded wall
//! thickness / lattice inflation) and [`Node::lerp`](crate::ops::Node::lerp)
//! (graded blends) — the same path `kernel-model`'s `LinearGrade` compiles to,
//! so a measured FEA field and a declarative linear law are interchangeable
//! grade sources.
//!
//! The closed loop: `ace_fea` → `stress_field.npy` →
//! `tools/stress_to_density.py` → `*_density.npy` →
//! [`GridField::from_npy_file`] → [`GridField::into_grade_law`] →
//! `Node::offset_by` → mesh. Thicker walls where the part works hardest.
//!
//! **What a `GridField` is NOT:** a signed distance field. Its values carry no
//! sign convention and no Lipschitz guarantee; it exists to *modulate*
//! geometry, not to be meshed directly. The Lipschitz contract of the
//! consuming operators (documented on `Node::offset_by`) is on the caller:
//! keep grade slopes gentle (a few % per mm) or mesh dense.
//!
//! **Grid frame:** `origin` is the world position of sample `(0, 0, 0)`;
//! sample `(i, j, k)` sits at `origin + cell·(i, j, k)` with one cubic `cell`
//! spacing on all axes. NOTE for `ace_fea` fields: its `origin_mm` names the
//! world position of grid NODE `(0,0,0)` while `stress_field.npy` values are
//! per-ELEMENT — pass `origin = origin_mm + cell/2·(1,1,1)` so samples land on
//! element centers.

use std::path::Path;
use std::sync::Arc;

use kernel_core::math::Vec3;

use crate::ops::ScalarField;

/// A scalar field sampled on a regular, axis-aligned grid, evaluated anywhere
/// in world space by trilinear interpolation (clamped at the borders: a query
/// outside the grid returns the nearest border value, so the field is total
/// and continuous over all of space).
///
/// Data layout is C-order over shape `(nx, ny, nz)` — the last axis (`z`)
/// varies fastest, exactly as NumPy writes a C-contiguous array — and the
/// world frame is documented on the [module](self).
#[derive(Clone)]
pub struct GridField {
	nx: usize,
	ny: usize,
	nz: usize,
	data: Vec<f32>,
	/// World position of sample `(0, 0, 0)`.
	pub origin: Vec3,
	/// Sample spacing (cubic, all axes), world units.
	pub cell: f32,
}

impl std::fmt::Debug for GridField {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		let (lo, hi) = self.value_range();
		f.debug_struct("GridField")
			.field("dims", &(self.nx, self.ny, self.nz))
			.field("origin", &self.origin)
			.field("cell", &self.cell)
			.field("value_range", &(lo, hi))
			.finish()
	}
}

impl GridField {
	/// Build a field from raw C-order values (shape `(nx, ny, nz)`, `z`
	/// fastest). Refuses zero-sized axes, a length mismatch, a non-positive or
	/// non-finite `cell`, a non-finite `origin`, and any non-finite value
	/// (one NaN would silently poison every trilinear sample near it — and a
	/// NaN grade offset survives `Node::offset_by`'s clamp, so refusing loudly
	/// here is what keeps the grading path honest).
	pub fn from_data(data: Vec<f32>, dims: (usize, usize, usize), origin: Vec3, cell: f32) -> Result<GridField, String> {
		let (nx, ny, nz) = dims;
		if nx == 0 || ny == 0 || nz == 0 {
			return Err(format!("GridField: every axis needs at least one sample, got dims ({nx}, {ny}, {nz})"));
		}
		let count = nx
			.checked_mul(ny)
			.and_then(|v| v.checked_mul(nz))
			.ok_or_else(|| format!("GridField: dims ({nx}, {ny}, {nz}) overflow usize"))?;
		if data.len() != count {
			return Err(format!("GridField: dims ({nx}, {ny}, {nz}) need {count} values, got {}", data.len()));
		}
		if !cell.is_finite() || cell <= 0.0 {
			return Err(format!("GridField: cell must be finite and > 0, got {cell}"));
		}
		if !origin.is_finite() {
			return Err(format!("GridField: origin must be finite, got {origin}"));
		}
		if let Some((i, v)) = data.iter().enumerate().find(|(_, v)| !v.is_finite()) {
			let bad = data.iter().filter(|v| !v.is_finite()).count();
			return Err(format!(
				"GridField: {bad} non-finite value(s), first at flat index {i} ({v}) — clean the field before grading (NaN/inf would poison interpolation)"
			));
		}
		Ok(GridField { nx, ny, nz, data, origin, cell })
	}

	/// Parse a NumPy `.npy` byte buffer into a field on the given world frame.
	///
	/// Accepted: magic `\x93NUMPY`, format versions 1–3, dtype `'<f4'` or
	/// `'<f8'` (f8 is narrowed to f32; a value beyond f32 range becomes ±inf
	/// and is then refused by the finite check), `fortran_order: False`
	/// (column-major files are REFUSED, never silently transposed — re-save
	/// with `numpy.ascontiguousarray(arr)`), shape exactly `(nx, ny, nz)`,
	/// C-order little-endian payload of exactly `nx·ny·nz` scalars.
	///
	/// The header is a Python dict literal; the hand parser here is small and
	/// tolerant of whitespace/key order but assumes the flat
	/// `{'descr': …, 'fortran_order': …, 'shape': (…)}` form NumPy itself
	/// writes. It does NOT evaluate Python: structured dtypes (a list-valued
	/// `descr`) fail the quoted-string scan and are refused with a clear
	/// error, and exotic hand-written headers may be refused too. No new
	/// dependencies — that narrowness is the documented trade.
	pub fn from_npy_bytes(bytes: &[u8], origin: Vec3, cell: f32) -> Result<GridField, String> {
		const MAGIC: &[u8] = b"\x93NUMPY";
		if bytes.len() < 8 || &bytes[..6] != MAGIC {
			return Err("not an NPY file: missing \\x93NUMPY magic".to_string());
		}
		let (major, minor) = (bytes[6], bytes[7]);
		let (header_start, header_len) = match major {
			1 => {
				if bytes.len() < 10 {
					return Err("truncated NPY: v1 header length missing".to_string());
				}
				(10usize, u16::from_le_bytes([bytes[8], bytes[9]]) as usize)
			}
			2 | 3 => {
				if bytes.len() < 12 {
					return Err(format!("truncated NPY: v{major} header length missing"));
				}
				(12usize, u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]) as usize)
			}
			_ => return Err(format!("unsupported NPY format version {major}.{minor} (this parser reads v1-v3)")),
		};
		let header_end = header_start
			.checked_add(header_len)
			.filter(|&e| e <= bytes.len())
			.ok_or_else(|| format!("truncated NPY: header claims {header_len} bytes but the buffer ends first"))?;
		let header = std::str::from_utf8(&bytes[header_start..header_end]).map_err(|_| "NPY header is not valid UTF-8".to_string())?;

		let descr = dict_quoted(header, "descr")?;
		let elem: usize = match descr.as_str() {
			"<f4" => 4,
			"<f8" => 8,
			other => return Err(format!("unsupported NPY dtype {other:?}: this parser reads little-endian scalar '<f4' or '<f8' only")),
		};
		if dict_bool(header, "fortran_order")? {
			return Err(
				"NPY has fortran_order=True (column-major): unsupported — this parser reads C-order payloads only; re-save with numpy.ascontiguousarray(arr)"
					.to_string(),
			);
		}
		let shape = dict_shape(header)?;
		if shape.len() != 3 {
			return Err(format!("expected a 3-D (nx, ny, nz) grid, got {}-D shape {:?}", shape.len(), shape));
		}
		let (nx, ny, nz) = (shape[0], shape[1], shape[2]);
		let count =
			nx.checked_mul(ny).and_then(|v| v.checked_mul(nz)).ok_or_else(|| format!("NPY shape ({nx}, {ny}, {nz}) overflows usize"))?;
		let need = count.checked_mul(elem).ok_or_else(|| format!("NPY payload size for shape ({nx}, {ny}, {nz}) overflows usize"))?;
		let payload = &bytes[header_end..];
		if payload.len() != need {
			return Err(format!("NPY payload is {} bytes but shape ({nx}, {ny}, {nz}) of '{descr}' needs exactly {need}", payload.len()));
		}
		let mut data = Vec::with_capacity(count);
		if elem == 4 {
			for c in payload.chunks_exact(4) {
				data.push(f32::from_le_bytes([c[0], c[1], c[2], c[3]]));
			}
		} else {
			for c in payload.chunks_exact(8) {
				let v = f64::from_le_bytes([c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7]]);
				data.push(v as f32);
			}
		}
		GridField::from_data(data, (nx, ny, nz), origin, cell)
	}

	/// [`GridField::from_npy_bytes`] on a file path (e.g. the
	/// `stress_field.npy` an `ace_fea` run wrote, or its
	/// `stress_to_density.py` remap).
	pub fn from_npy_file(path: impl AsRef<Path>, origin: Vec3, cell: f32) -> Result<GridField, String> {
		let path = path.as_ref();
		let bytes = std::fs::read(path).map_err(|e| format!("cannot read {}: {e}", path.display()))?;
		GridField::from_npy_bytes(&bytes, origin, cell).map_err(|e| format!("{}: {e}", path.display()))
	}

	/// Grid dimensions `(nx, ny, nz)`.
	pub fn dims(&self) -> (usize, usize, usize) {
		(self.nx, self.ny, self.nz)
	}

	/// `(min, max)` over the stored values — the natural inputs to
	/// [`GridField::normalized`] when no external scale (e.g. an allowable
	/// stress) is given.
	pub fn value_range(&self) -> (f32, f32) {
		let mut lo = f32::INFINITY;
		let mut hi = f32::NEG_INFINITY;
		for &v in &self.data {
			lo = lo.min(v);
			hi = hi.max(v);
		}
		(lo, hi)
	}

	/// Trilinear sample at world point `p`, clamped at the borders: outside
	/// the grid the nearest border value extends unchanged (constant
	/// extrapolation), so a grade law built from this field stays bounded and
	/// continuous even where a mesher pads its domain past the data.
	pub fn sample(&self, p: Vec3) -> f32 {
		let l = (p - self.origin) / self.cell;
		let (x0, x1, tx) = axis_clamped(l.x, self.nx);
		let (y0, y1, ty) = axis_clamped(l.y, self.ny);
		let (z0, z1, tz) = axis_clamped(l.z, self.nz);
		let at = |ix: usize, iy: usize, iz: usize| self.data[(ix * self.ny + iy) * self.nz + iz];
		let lerp = |a: f32, b: f32, t: f32| a + (b - a) * t;
		let c00 = lerp(at(x0, y0, z0), at(x1, y0, z0), tx);
		let c10 = lerp(at(x0, y1, z0), at(x1, y1, z0), tx);
		let c01 = lerp(at(x0, y0, z1), at(x1, y0, z1), tx);
		let c11 = lerp(at(x0, y1, z1), at(x1, y1, z1), tx);
		lerp(lerp(c00, c10, ty), lerp(c01, c11, ty), tz)
	}

	/// Affine remap of the values onto `[0, 1]`, clamped: `lo → 0`, `hi → 1`,
	/// everything outside pinned to the ends. The grid frame is unchanged.
	/// Typical use: `field.normalized(0.0, allowable_stress)` before
	/// [`GridField::into_grade_law`]. Panics (contract violation, matching
	/// `Node::offset_by`'s style) if `lo`/`hi` are non-finite or `hi <= lo`.
	pub fn normalized(&self, lo: f32, hi: f32) -> GridField {
		assert!(lo.is_finite() && hi.is_finite() && hi > lo, "GridField::normalized: need finite lo < hi, got lo={lo} hi={hi}");
		let inv = 1.0 / (hi - lo);
		GridField {
			nx: self.nx,
			ny: self.ny,
			nz: self.nz,
			data: self.data.iter().map(|&v| ((v - lo) * inv).clamp(0.0, 1.0)).collect(),
			origin: self.origin,
			cell: self.cell,
		}
	}

	/// The raw field as a [`ScalarField`] — the exact closure type
	/// [`Node::offset_by`](crate::ops::Node::offset_by) and
	/// [`Node::lerp`](crate::ops::Node::lerp) consume (and the same shape
	/// `kernel-model` compiles its `LinearGrade` into). Values pass through
	/// unmapped; use [`GridField::into_grade_law`] when the grid holds a
	/// normalized density that still needs turning into millimetre offsets.
	pub fn into_scalar_field(self) -> ScalarField {
		Arc::new(move |p: Vec3| self.sample(p))
	}

	/// A grid-backed **grade law**: the sampled value is clamped to `[0, 1]`
	/// (it should already be a density — see [`GridField::normalized`] /
	/// `tools/stress_to_density.py`) and mapped affinely to a surface offset,
	/// `0 → at_zero`, `1 → at_one` (world units; positive inflates). This is
	/// the FEA-driven counterpart of `kernel-model`'s `LinearGrade`, ready for
	/// `Node::offset_by(law, max_abs)`. Panics on non-finite endpoints.
	///
	/// Worked example — a density ramp thickening a gyroid along +x, the
	/// stress-driven graded-lattice loop in miniature:
	///
	/// ```
	/// use kernel_implicit::grid_field::GridField;
	/// use kernel_implicit::{Aabb, Gyroid, Node, Sdf, Vec3};
	///
	/// // Two density samples 30 mm apart: 0.0 at x=0 (idle) → 1.0 at x=30 (hot).
	/// // In production this grid comes from GridField::from_npy_file on the
	/// // *_density.npy that tools/stress_to_density.py derived from an ace_fea
	/// // stress_field.npy.
	/// let density = GridField::from_data(vec![0.0, 1.0], (2, 1, 1), Vec3::ZERO, 30.0).unwrap();
	/// let law = density.clone().into_grade_law(-0.25, 0.25); // mm: thin idle walls, fat hot walls
	/// assert_eq!(
	///     (law(Vec3::ZERO), law(Vec3::new(15.0, 0.0, 0.0)), law(Vec3::new(30.0, 0.0, 0.0))),
	///     (-0.25, 0.0, 0.25),
	/// );
	/// // Feed the kernel's existing grading operator — same path LinearGrade uses.
	/// let region = Aabb::new(Vec3::ZERO, Vec3::new(30.0, 30.0, 20.0));
	/// let graded = Node::primitive_bound(Gyroid::new(region, 0.55, 1.3)).offset_by(law, 0.3);
	/// assert!(graded.bounds().max.x >= 30.0);
	/// ```
	pub fn into_grade_law(self, at_zero: f32, at_one: f32) -> ScalarField {
		assert!(
			at_zero.is_finite() && at_one.is_finite(),
			"GridField::into_grade_law: offsets must be finite, got at_zero={at_zero} at_one={at_one}"
		);
		Arc::new(move |p: Vec3| {
			let t = self.sample(p).clamp(0.0, 1.0);
			at_zero + (at_one - at_zero) * t
		})
	}
}

/// Clamp a continuous sample coordinate onto axis `[0, n-1]` and split it into
/// the two bracketing indices plus the interpolation fraction. At or beyond
/// the last sample the pair degenerates to `(n-1, n-1, 0)` — that degeneracy
/// IS the border clamp (and makes 1-sample axes constant along that axis).
fn axis_clamped(l: f32, n: usize) -> (usize, usize, f32) {
	let c = l.clamp(0.0, (n - 1) as f32);
	let i0 = (c.floor() as usize).min(n - 1);
	let i1 = (i0 + 1).min(n - 1);
	(i0, i1, c - i0 as f32)
}

/// Locate `'key'` in the header dict and return the raw text after its `:`.
fn dict_value<'a>(header: &'a str, key: &str) -> Result<&'a str, String> {
	let pos = header
		.find(&format!("'{key}'"))
		.or_else(|| header.find(&format!("\"{key}\"")))
		.ok_or_else(|| format!("NPY header has no {key:?} key in {header:?}"))?;
	let after = &header[pos + key.len() + 2..];
	let colon = after.find(':').ok_or_else(|| format!("NPY header {key:?} has no ':' after it in {header:?}"))?;
	Ok(after[colon + 1..].trim_start())
}

/// A quoted-string dict value (used for `descr`). A non-string value — e.g. a
/// structured dtype's list — is refused here.
fn dict_quoted(header: &str, key: &str) -> Result<String, String> {
	let v = dict_value(header, key)?;
	let quote = v.chars().next().ok_or_else(|| format!("NPY header {key:?} value is empty"))?;
	if quote != '\'' && quote != '"' {
		return Err(format!("NPY header {key:?} value is not a plain quoted string (structured dtypes are unsupported): {v:?}"));
	}
	let rest = &v[1..];
	let end = rest.find(quote).ok_or_else(|| format!("NPY header {key:?} string is unterminated: {v:?}"))?;
	Ok(rest[..end].to_string())
}

/// A `True`/`False` dict value (used for `fortran_order`).
fn dict_bool(header: &str, key: &str) -> Result<bool, String> {
	let v = dict_value(header, key)?;
	if v.starts_with("True") {
		Ok(true)
	} else if v.starts_with("False") {
		Ok(false)
	} else {
		Err(format!("NPY header {key:?} is neither True nor False: {v:?}"))
	}
}

/// The `shape` tuple as dimensions (tolerates the trailing comma NumPy writes
/// in 1-tuples).
fn dict_shape(header: &str) -> Result<Vec<usize>, String> {
	let v = dict_value(header, "shape")?;
	let open = v.strip_prefix('(').ok_or_else(|| format!("NPY header \"shape\" is not a tuple: {v:?}"))?;
	let end = open.find(')').ok_or_else(|| format!("NPY header \"shape\" tuple is unterminated: {v:?}"))?;
	open[..end]
		.split(',')
		.map(str::trim)
		.filter(|s| !s.is_empty())
		.map(|s| s.parse::<usize>().map_err(|_| format!("NPY header \"shape\" has a non-integer dimension {s:?}")))
		.collect()
}
