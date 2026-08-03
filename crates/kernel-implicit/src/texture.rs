// Copyright (c) LMCAD. Licensed under the MIT License.

//! Surface-texture **displacement fields**: knurl, stipple and deterministic
//! value-noise, applied to any CSG [`Node`] as `d′(p) = d(p) − amplitude·t(p)`
//! with `t(p) ∈ [0, 1]` — the grip/anti-slip/organic-finish operator of the
//! implicit half (a texture is a few lines of field algebra here, where the
//! B-rep half would need thousands of tiny boolean features).
//!
//! # The Lipschitz algebra (load-bearing for narrow-band meshing)
//!
//! Displacing a distance field steepens it: if the base field is
//! `L_base`-Lipschitz and the texture field is `L_t`-Lipschitz, the raw
//! displaced field `d − a·t` is at worst `L′ = L_base + |a|·L_t`-Lipschitz.
//! The narrow-band meshers ([`crate::narrow_band`]) prune blocks assuming a
//! field never overstates the distance to its zero set — sound only for
//! ≤ 1-Lipschitz fields. So, exactly like [`crate::Gyroid`] / [`crate::Tpms`]
//! normalize their sine fields, [`Displaced`] **divides by the derived bound
//! `L′`** (taking `L_base = 1`, see below): the emitted field is ≤ 1-Lipschitz
//! by construction, the zero set — and therefore the meshed geometry — is
//! unchanged, and narrow-band pruning stays sound. Each texture's `L_t` is
//! derived (not guessed) in [`Texture::lipschitz`]'s docs, and the test suite
//! probes the sampled gradient against every declared bound.
//!
//! **Contract on the base** (same honesty as [`Node::offset_by`]): the divisor
//! assumes `L_base = 1`, which holds for every exact primitive, `min`/`max`
//! CSG of them, smooth blends, patterns, transforms, and the normalized
//! TPMS/lattice fields — everything the narrow-band mesher already accepts. A
//! base whose own field is steeper than 1-Lipschitz (an `offset_by`/`lerp`
//! with a steep modulation field) keeps that excess: the quotient understates
//! it, so mesh such a tree densely (`surface_nets` / `manifold_dual_contour`),
//! exactly as those operators already document.
//!
//! The result is wrapped via [`Node::primitive_bound`]: a displaced distance
//! is a [`crate::FieldQuality::DistanceBound`], not an exact SDF, so a
//! downstream `offset`/`shell` is honestly flagged approximate.
//!
//! ```
//! use kernel_implicit::texture::{displaced, Texture};
//! use kernel_implicit::{Cylinder, Node, Sdf, Vec3};
//!
//! // A Ø16 grip post with a 2 mm crossed knurl (amplitude 0.4 mm,
//! // peak-to-valley 0.2 mm — see Texture::Knurl on the gyroid range).
//! let post = Node::primitive(Cylinder::new(Vec3::ZERO, Vec3::new(0.0, 0.0, 30.0), 8.0));
//! let grip = displaced(post, 0.4, Texture::Knurl { pitch: 2.0, depth_frac: 1.0 });
//! assert!(grip.bounds().max.x >= 8.0); // bounds grew by the outward amplitude
//! ```

use kernel_core::math::{Aabb, DVec3, Vec3};
use kernel_core::sdf::Sdf;

use crate::ops::Node;

/// Fraction of the cell edge a stipple dome may be jittered per axis. Chosen
/// with [`STIPPLE_R0_FRAC`] so `jitter + radius = 0.5` — a dome never leaves
/// its own cell, so evaluating only the query point's cell is exact and the
/// field stays continuous (domes fade to 0 exactly at their rim).
const STIPPLE_JITTER_FRAC: f32 = 0.15;

/// Stipple dome radius as a fraction of the cell edge (see above).
const STIPPLE_R0_FRAC: f32 = 0.35;

/// The six ±45° unit diagonals of the three coordinate planes — the knurl's
/// grating directions (world-space triplanar-ish: every plane of the trio
/// carries a crossed ridge pair, so the pattern reads as a knurl on any
/// surface orientation without needing the surface normal).
const KNURL_DIAGS: [Vec3; 6] = {
	const H: f32 = std::f32::consts::FRAC_1_SQRT_2;
	[
		Vec3::new(H, H, 0.0),
		Vec3::new(H, -H, 0.0),
		Vec3::new(0.0, H, H),
		Vec3::new(0.0, H, -H),
		Vec3::new(H, 0.0, H),
		Vec3::new(-H, 0.0, H),
	]
};

/// A procedural surface texture: a scalar field `t(p) ∈ [0, 1]` over world
/// space, meant to displace a solid via [`displaced`] (`d − amplitude·t`).
/// All three variants are **deterministic** — same inputs, bit-identical
/// field, run to run and machine to machine (the hashed variants use a pure
/// integer splitmix64-style hash, no RNG state; see [`lattice_hash01`]'s doc).
#[derive(Clone, Copy, Debug)]
pub enum Texture {
	/// Crossed ±45° sinusoid ridges — the classic machinist's knurl.
	///
	/// `t = depth_frac · (1 + g)/2` with `g` the average of six sinusoids
	/// `sin(2π/pitch · ⟨d̂ᵢ, p⟩)` along the ±45° diagonals of the xy/yz/zx
	/// planes (world-space "triplanar-ish": orientation-free, no surface
	/// normal needed). `pitch` is the spatial period of each grating (> 0);
	/// `depth_frac ∈ [0, 1]` scales ridge contrast.
	///
	/// Honest range (pinned by test): by the product-to-sum identity each
	/// crossed pair collapses — `sin(k·u) + sin(k·v) = 2·sin(k·x̃)·cos(k·ỹ)`
	/// with `x̃ = x/√2` — so `g` is exactly a **gyroid** field in `1/√2`-scaled
	/// coordinates, `g ∈ [−½, ½]`, hence `t ∈ [¼, ¾]·depth_frac` and the world
	/// peak-to-valley ridge depth is `amplitude·depth_frac/2` (the crossed
	/// gratings genuinely interfere; that is the pattern, not a loss).
	Knurl { pitch: f32, depth_frac: f32 },
	/// Raised dots: space is tiled into `cell`-sized cubes; a per-cell hash
	/// picks `coverage` of the cells (`∈ [0, 1]`) to carry one smooth dome
	/// `t = (1 − (r/r₀)²)²` of radius `r₀ = 0.35·cell`, its center jittered
	/// up to `±0.15·cell` per axis by the same hash. Domes never leave their
	/// cell (0.15 + 0.35 = 0.5), so the field is continuous (0 at every rim)
	/// and single-cell evaluation is exact.
	Stipple { cell: f32, coverage: f32 },
	/// Trilinear value-noise: hash values in `[0, 1)` on the integer lattice
	/// of spacing `cell`, trilinearly interpolated — C⁰, deterministic, and
	/// seeded (`seed` selects the lattice values, nothing else).
	Noise { cell: f32, seed: u32 },
}

impl Texture {
	/// Panic loudly on non-physical parameters (mirrors [`crate::ExprSdf::new`]'s
	/// contract style): `pitch`/`cell` must be finite and > 0, `depth_frac` and
	/// `coverage` finite in `[0, 1]`.
	fn validate(&self) {
		match *self {
			Texture::Knurl { pitch, depth_frac } => {
				assert!(pitch.is_finite() && pitch > 0.0, "Texture::Knurl: pitch must be finite and > 0, got {pitch}");
				assert!(
					depth_frac.is_finite() && (0.0..=1.0).contains(&depth_frac),
					"Texture::Knurl: depth_frac must be in [0, 1], got {depth_frac}"
				);
			}
			Texture::Stipple { cell, coverage } => {
				assert!(cell.is_finite() && cell > 0.0, "Texture::Stipple: cell must be finite and > 0, got {cell}");
				assert!(
					coverage.is_finite() && (0.0..=1.0).contains(&coverage),
					"Texture::Stipple: coverage must be in [0, 1], got {coverage}"
				);
			}
			Texture::Noise { cell, .. } => {
				assert!(cell.is_finite() && cell > 0.0, "Texture::Noise: cell must be finite and > 0, got {cell}");
			}
		}
	}

	/// The texture field `t(p) ∈ [0, 1]` (clamped against last-ulp rounding).
	pub fn value(&self, p: Vec3) -> f32 {
		let t = match *self {
			Texture::Knurl { pitch, depth_frac } => {
				let k = std::f32::consts::TAU / pitch;
				let g: f32 = KNURL_DIAGS.iter().map(|d| (k * p.dot(*d)).sin()).sum::<f32>() / 6.0;
				0.5 * depth_frac * (1.0 + g)
			}
			Texture::Stipple { cell, coverage } => {
				stipple_value(p.x as f64, p.y as f64, p.z as f64, cell as f64, coverage as f64) as f32
			}
			Texture::Noise { cell, seed } => noise_value(p.x as f64, p.y as f64, p.z as f64, cell as f64, seed) as f32,
		};
		t.clamp(0.0, 1.0)
	}

	/// Double-precision [`Texture::value`] — same formulas evaluated in `f64`,
	/// with the SAME integer lattice hashes (bit-identical cell values), so the
	/// texture stays coherent on the [`Sdf::distance64`] path of large parts.
	pub fn value64(&self, p: DVec3) -> f64 {
		let t = match *self {
			Texture::Knurl { pitch, depth_frac } => {
				let k = std::f64::consts::TAU / pitch as f64;
				let g: f64 = KNURL_DIAGS.iter().map(|d| (k * p.dot(d.as_dvec3())).sin()).sum::<f64>() / 6.0;
				0.5 * depth_frac as f64 * (1.0 + g)
			}
			Texture::Stipple { cell, coverage } => stipple_value(p.x, p.y, p.z, cell as f64, coverage as f64),
			Texture::Noise { cell, seed } => noise_value(p.x, p.y, p.z, cell as f64, seed),
		};
		t.clamp(0.0, 1.0)
	}

	/// The texture's **derived** spatial Lipschitz bound `L_t ≥ sup |∇t|`
	/// (world units⁻¹) — the growth term of the displaced field's bound
	/// `L′ = 1 + |amplitude|·L_t`. Derivations (each pinned by a sampled-
	/// gradient probe in `tests/texture_text.rs`):
	///
	/// - **Knurl**: `t = ½·depth_frac·(1 + ⅙·Σᵢ sin(k⟨d̂ᵢ,p⟩))`, `k = 2π/pitch`.
	///   Each sinusoid along a unit direction is `k`-Lipschitz, so by the
	///   triangle inequality `|∇t| ≤ ½·depth_frac·(⅙·6k) = depth_frac·π/pitch`
	///   — the DECLARED bound (simple, provably safe). Via the gyroid identity
	///   (see [`Texture::Knurl`]) the tight constant is `depth_frac·π/(√6·pitch)`
	///   ≈ 0.41× the declaration; over-declaring only pads the normalizer,
	///   never the geometry.
	/// - **Stipple**: inside a dome `t(r) = (1 − (r/r₀)²)²`, so `|dt/dr| =
	///   4(r/r₀)(1 − (r/r₀)²)/r₀`, maximal at `r/r₀ = 1/√3` where it equals
	///   `8/(3√3·r₀)`; with `r₀ = 0.35·cell` that is `≈ 4.400/cell`. Domes are
	///   disjoint and fade to 0 at their rim, so the per-dome bound is global.
	/// - **Noise**: within a lattice cell trilinear interpolation is affine in
	///   each axis with per-axis slope ≤ (max corner delta)/cell ≤ `1/cell`
	///   (values lie in `[0, 1)`), so `|∇t| ≤ √3·max_delta/cell ≤ √3/cell`.
	///   Continuity across cell faces (shared corners) makes the bound global.
	pub fn lipschitz(&self) -> f32 {
		match *self {
			Texture::Knurl { pitch, depth_frac } => depth_frac * std::f32::consts::PI / pitch,
			Texture::Stipple { cell, .. } => 8.0 / (3.0 * 3.0_f32.sqrt() * STIPPLE_R0_FRAC * cell),
			Texture::Noise { cell, .. } => 3.0_f32.sqrt() / cell,
		}
	}
}

/// `splitmix64` finalizer (Steele, Lea & Flood, *Fast Splittable Pseudorandom
/// Number Generators*; the widely used public-domain constants) — a bijective
/// avalanche mix on 64 bits. Pure integer ops: deterministic on every
/// platform, no RNG state, no dependency.
#[inline]
fn splitmix64(mut x: u64) -> u64 {
	x = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
	x = (x ^ (x >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
	x = (x ^ (x >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
	x ^ (x >> 31)
}

/// Deterministic hash of an integer lattice point (plus a stream `salt`) to
/// `[0, 1)`: the coordinates are spread by odd 64-bit multipliers (golden
/// ratio / xxhash primes), XOR-combined with the salt, avalanched through
/// [`splitmix64`], and the top 24 bits become a dyadic fraction (exact in
/// `f32`, so `f32` and `f64` paths see bit-identical lattice values).
#[inline]
fn lattice_hash01(i: i64, j: i64, k: i64, salt: u64) -> f32 {
	let key = (i as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15)
		^ (j as u64).wrapping_mul(0xC2B2_AE3D_27D4_EB4F)
		^ (k as u64).wrapping_mul(0x1656_67B1_9E37_79F9)
		^ salt;
	((splitmix64(key) >> 40) as f32) * (1.0 / 16_777_216.0)
}

/// Stipple field (shared by the `f32`/`f64` paths — geometry in `f64`, hashes
/// on integers). See [`Texture::Stipple`] for the construction.
fn stipple_value(x: f64, y: f64, z: f64, cell: f64, coverage: f64) -> f64 {
	let (i, j, k) = ((x / cell).floor(), (y / cell).floor(), (z / cell).floor());
	let (ii, jj, kk) = (i as i64, j as i64, k as i64);
	// Streams: 0 = occupancy, 1..=3 = per-axis center jitter.
	if lattice_hash01(ii, jj, kk, 0) as f64 >= coverage {
		return 0.0;
	}
	let jit = |salt: u64| (lattice_hash01(ii, jj, kk, salt) as f64 - 0.5) * 2.0 * STIPPLE_JITTER_FRAC as f64 * cell;
	let cx = (i + 0.5) * cell + jit(1);
	let cy = (j + 0.5) * cell + jit(2);
	let cz = (k + 0.5) * cell + jit(3);
	let r0 = STIPPLE_R0_FRAC as f64 * cell;
	let r2 = ((x - cx) * (x - cx) + (y - cy) * (y - cy) + (z - cz) * (z - cz)) / (r0 * r0);
	if r2 >= 1.0 {
		0.0
	} else {
		(1.0 - r2) * (1.0 - r2)
	}
}

/// Trilinear value-noise (shared `f32`/`f64` core). The user `seed` is
/// avalanched once and XORed into every lattice hash, so different seeds give
/// statistically unrelated lattices while staying fully deterministic.
fn noise_value(x: f64, y: f64, z: f64, cell: f64, seed: u32) -> f64 {
	let (qx, qy, qz) = (x / cell, y / cell, z / cell);
	let (fx, fy, fz) = (qx.floor(), qy.floor(), qz.floor());
	let (i, j, k) = (fx as i64, fy as i64, fz as i64);
	let (tx, ty, tz) = (qx - fx, qy - fy, qz - fz);
	let salt = splitmix64(seed as u64) | 1;
	let v = |di: i64, dj: i64, dk: i64| lattice_hash01(i + di, j + dj, k + dk, salt) as f64;
	let lerp = |a: f64, b: f64, t: f64| a + (b - a) * t;
	let c00 = lerp(v(0, 0, 0), v(1, 0, 0), tx);
	let c10 = lerp(v(0, 1, 0), v(1, 1, 0), tx);
	let c01 = lerp(v(0, 0, 1), v(1, 0, 1), tx);
	let c11 = lerp(v(0, 1, 1), v(1, 1, 1), tx);
	lerp(lerp(c00, c10, ty), lerp(c01, c11, ty), tz)
}

/// A base [`Sdf`] displaced by a [`Texture`]: raw field `d(p) − amplitude·t(p)`,
/// emitted **divided by the declared bound** `L′ = 1 + |amplitude|·L_t` so the
/// result stays ≤ 1-Lipschitz for a ≤ 1-Lipschitz base (the module docs derive
/// the algebra and its honest contract). Positive `amplitude` raises the
/// texture proud of the surface (grip ridges/dots); negative recesses it.
///
/// The zero set is untouched by the division, so the meshed geometry is
/// exactly the raw displaced surface. Use [`displaced`] for the composable
/// [`Node`] form; this struct is the reusable leaf (any `S: Sdf` base).
pub struct Displaced<S: Sdf> {
	base: S,
	amplitude: f32,
	texture: Texture,
	/// `L′ = 1 + |amplitude|·L_t` — the declared raw-field bound, also the
	/// normalizer.
	l_total: f32,
}

impl<S: Sdf> Displaced<S> {
	/// Displace `base` by `amplitude · texture(p)` (world units; finite,
	/// either sign — asserted, as are the texture parameters).
	pub fn new(base: S, amplitude: f32, texture: Texture) -> Self {
		assert!(amplitude.is_finite(), "Displaced: amplitude must be finite, got {amplitude}");
		texture.validate();
		let l_total = 1.0 + amplitude.abs() * texture.lipschitz();
		Self { base, amplitude, texture, l_total }
	}

	/// The declared Lipschitz bound `L′ = 1 + |amplitude|·L_t` of the RAW
	/// displaced field — the constant the emitted field is divided by. The
	/// emitted [`Sdf::distance`] is therefore declared ≤ 1-Lipschitz (for a
	/// ≤ 1-Lipschitz base), which is what the tests probe.
	pub fn lipschitz_normalizer(&self) -> f32 {
		self.l_total
	}
}

impl<S: Sdf> Sdf for Displaced<S> {
	fn distance(&self, p: Vec3) -> f32 {
		(self.base.distance(p) - self.amplitude * self.texture.value(p)) / self.l_total
	}

	fn distance64(&self, p: DVec3) -> f64 {
		(self.base.distance64(p) - self.amplitude as f64 * self.texture.value64(p)) / self.l_total as f64
	}

	fn bounds(&self) -> Aabb {
		// t ∈ [0, 1]: the surface sits where d = amplitude·t, i.e. at most
		// `amplitude` OUTSIDE the base surface (positive amplitude only —
		// a negative amplitude only recesses, which the base bound contains).
		self.base.bounds().pad(self.amplitude.max(0.0))
	}
}

/// Displace the surface of `base` by a procedural [`Texture`]:
/// `d′(p) = d(p) − amplitude·t(p)`, `t ∈ [0, 1]` — knurl a grip, stipple a
/// handle, roughen a casting. Returns a [`Node`] (wrapped via
/// [`Node::primitive_bound`] — a displaced distance is a
/// [`crate::FieldQuality::DistanceBound`]) so it composes with every CSG
/// combinator, and the emitted field is normalized to stay ≤ 1-Lipschitz for
/// any ≤ 1-Lipschitz `base` (module docs: the bound algebra, the derived
/// per-texture `L_t`, and the `offset_by`/`lerp` caveat).
pub fn displaced(base: Node, amplitude: f32, texture: Texture) -> Node {
	Node::primitive_bound(Displaced::new(base, amplitude, texture))
}
