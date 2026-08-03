// Copyright (c) LMCAD. Licensed under the MIT License.

//! GPU **narrow-band** Surface Nets: band-limited extraction whose GPU work
//! scales with the surface-straddling blocks, not the dense lattice.
//!
//! [`crate::GpuSurfaceNets`] samples the WHOLE padded lattice — `O(n³)` field
//! evaluations even though the iso-surface is a 2-manifold whose cost should
//! scale with area. This module closes the ledgered gap (the GPU analogue of
//! `kernel_implicit::narrow_band`): a coarse Lipschitz-safe block scan flags
//! the blocks that can contain surface, a prefix-sum compaction builds a dense
//! active-block list, and refine passes run the SAME Surface Nets cell rules
//! as the dense extractor on active blocks only. Domains beyond the dense
//! meshers' 2²⁸-cell cap are ACCEPTED here (that is the point); the ceiling is
//! a 2⁴⁴ conceptual-lattice cap (mirroring the CPU tracker's
//! `NARROWBAND_MAX_LATTICE_CELLS`), 2²⁴ points per axis (f32-exact lattice
//! coordinates), and the device's buffer limits on the block grid — all
//! refused with a loud [`GpuError::TooLarge`], never a silent empty mesh.
//!
//! # Honesty contract (same as the rest of kernel-gpu — `NUMERICS.md` §"GPU
//! evaluation and extraction")
//!
//! **The CPU stays bit-authoritative**; the GPU is the tolerance-equivalent
//! (`|gpu − cpu_f32| ≤ 1e-4·(1+|cpu_f32|)`) preview/bulk path. This extractor
//! is a *preview/bulk* mesher exactly like the dense one: closed output by
//! construction, but the watertight **authority** for production remains the
//! CPU's `manifold_dual_contour`, and `check_mesh` gates either way.
//!
//! # Pipeline
//!
//! 1. **COARSE** — one field sample per `B³`-cell block (B = 4) at the block's
//!    integer centre lattice point; flag active iff `|d(centre)| ≤ band_eff +
//!    reach`, `reach` = exact centre-to-farthest-block-corner distance.
//! 2. **COMPACT** — exclusive prefix sum over the flags ([`crate::scan`], the
//!    same scan the dense extractor uses) + a scatter pass building the dense
//!    active-block list and the inverse block→slot map.
//! 3. **REFINE** — for active blocks only: corner sampling ((B+1)³ points per
//!    block), then the dense extractor's cell passes verbatim (same
//!    `kernel_core::marching::edge_tables` unroll spliced by the same
//!    generator, same vertex placement, same quad conditions and orientation),
//!    with vertex/quad slots from prefix sums over the compacted active cells.
//!
//! # The Lipschitz-safe coarse test
//!
//! Field trees are ≤ 1-Lipschitz by the `FieldQuality` contract (both
//! `ExactSdf` and `DistanceBound` declare a 1-Lipschitz field — e.g. `Gyroid`
//! divides by `√3·scale` precisely to stay ≤ 1 for narrow-band pruning). A
//! straddling cell in a block implies a surface point inside the block
//! (continuity across the sign change), hence `|d(centre)| ≤ reach` — so the
//! test can never leave a straddling cell in an unflagged block. The band
//! floor `band_eff = band.max(2·voxel)` (the DOCUMENTED CLAMP, pinned in
//! `tests/narrow_band.rs` — `band` of 0, negative, or NaN clamps UP to the
//! floor) absorbs what exact arithmetic does not: (a) f32 evaluation noise,
//! including coarse-vs-refine cross-pipeline rounding, bounded by the declared
//! GPU tolerance (≪ a voxel at part scale), and (b) the mild > 1 Lipschitz
//! excess the pairwise seam combinators (fillet/chamfer/smooth-k) can reach
//! under parallel operand gradients (≤ √2 · half-diagonal < 2 voxels at
//! B = 4). Caveat mirrored from the CPU tracker: a heavily non-metric field —
//! `offset_by`/`lerp` with a steep modulation field, an `Expr` leaf with an
//! understated Lipschitz bound — can defeat ANY Lipschitz pruning; redistance
//! first, or raise `band` (it adds directly to the flag threshold and is the
//! caller's margin lever).
//!
//! # Boundary-sample consistency (why the band-limited mesh still closes)
//!
//! Design choice: **per-block duplicated evaluation of identical
//! coordinates**, not a shared sparse sample buffer. Each active block samples
//! its own (B+1)³ corner grid, so a lattice point on a face shared by two
//! active blocks is evaluated once per block. Closure survives because the
//! duplicates are bit-identical: the world coordinate comes from ONE
//! expression over the GLOBAL point index (`lm_nb_point`, never a function of
//! which block asked) and the sample from the single `lm_field` call site in
//! the corner pass — an invocation-independent expression (no workgroup
//! memory, no atomics, no derivatives), so equal global indices produce equal
//! f32 bits in every workgroup that evaluates them. Neighbouring cells across
//! a block face therefore see identical corner values ⇒ identical straddle
//! masks, and the dense extractor's shared-corner-buffer closure argument
//! carries over verbatim. Cell flags and vertex ids are never duplicated
//! (each cell belongs to exactly one block); cross-block quad stitching looks
//! neighbour flags/ids up through the block slot map, and every cell incident
//! to a straddling minimal edge lies in an active block by the Lipschitz
//! argument, so no quad is dropped at a block boundary.
//!
//! # Determinism
//!
//! Vertex ids and triangle slots come from exclusive prefix sums over the
//! (block, cell) order — no atomics anywhere — so extraction is
//! invocation-order-independent and bit-stable run to run on the same
//! device/driver: the same statement the dense extractor makes, pinned in
//! `tests/narrow_band.rs`.
//!
//! # Work accounting
//!
//! [`NarrowBandStats`] counts FIELD samples: one per block (coarse) plus each
//! refined block's in-lattice corner samples. Per-vertex normal gradients
//! (6 evaluations each) are excluded on BOTH paths — the dense extractor does
//! the identical per-emitted-vertex work. `dense_samples` is the corner-sample
//! count the dense extractor would evaluate on the IDENTICAL lattice: the
//! honest denominator for the sparsity receipts pinned in the tests.

use kernel_core::math::{Aabb, Vec3};
use kernel_core::mesh::Mesh;
use kernel_core::mesher::Resolution;
use wgpu::util::DeviceExt;

use crate::codegen::lower;
use crate::extract::edge_unroll;
use crate::scan::GpuScan;
use crate::tree::GpuNode;
use crate::{GpuContext, GpuError};

/// Cells per block axis. B = 4 keeps the refined band thin (block
/// half-diagonal 2√3 ≈ 3.46 voxels) while the coarse pass stays 64× sparser
/// than the cell lattice; the per-block corner grid is (B+1)³ = 125 points.
const B: u32 = 4;
const B_CELLS: u64 = (B as u64) * (B as u64) * (B as u64);
const B_PTS: u64 = (B as u64 + 1) * (B as u64 + 1) * (B as u64 + 1);

/// The documented band floor, in voxels (see module docs: f32 rounding + the
/// seam combinators' √2 worst case). `band` below `2·voxel` clamps UP to it.
const MIN_BAND_VOXELS: f32 = 2.0;

/// Conceptual-lattice cap, mirroring the CPU tracker's
/// `NARROWBAND_MAX_LATTICE_CELLS` (2⁴⁴): the narrow band never materializes
/// the point lattice, so the conceptual count only has to keep index
/// arithmetic sane; real memory scales with blocks (÷64) and active cells.
const NB_MAX_LATTICE_CELLS: f64 = (1u64 << 44) as f64;

/// Per-axis point cap so `f32(index)` is exact in the shaders (2²⁴).
const MAX_AXIS_POINTS: f64 = (1u64 << 24) as f64;

/// The honest work receipt of a narrow-band extraction.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct NarrowBandStats {
	/// Blocks whose coarse test flagged possible surface (these were refined).
	pub active_blocks: u64,
	/// Blocks in the full block grid (= coarse-pass field samples).
	pub total_blocks: u64,
	/// Field samples actually evaluated: `total_blocks` centre samples plus
	/// each active block's in-lattice corner samples. Per-vertex normal
	/// gradients (6 evals each, identical on the dense path) are excluded.
	pub samples_evaluated: u64,
	/// Corner samples the DENSE extractor would evaluate on the identical
	/// lattice (`nx·ny·nz`) — the denominator for sparsity receipts.
	pub dense_samples: u64,
}

/// The extraction entry points, appended to a generated field module (same
/// assembly scheme as the dense extractor: `__EDGE_UNROLL__` receives the
/// SAME `extract::edge_unroll()` splice, so cell topology/vertex placement
/// are shared by construction).
const NB_TEMPLATE: &str = r#"
struct LmNbParams {
	cdims: vec4u,
	bdims: vec4u,
	origin: vec4f,
	band: vec4f,
}
@group(1) @binding(0) var<uniform> lm_np: LmNbParams;
@group(1) @binding(1) var<storage, read_write> lm_nb_bflag: array<u32>;
@group(1) @binding(2) var<storage, read> lm_nb_bscan: array<u32>;
@group(1) @binding(3) var<storage, read_write> lm_nb_bslot: array<u32>;
@group(1) @binding(4) var<storage, read_write> lm_nb_active: array<u32>;
@group(1) @binding(5) var<storage, read_write> lm_nb_corners: array<f32>;
@group(1) @binding(6) var<storage, read_write> lm_nb_maskflag: array<u32>;
@group(1) @binding(7) var<storage, read_write> lm_nb_vflag: array<u32>;
@group(1) @binding(8) var<storage, read_write> lm_nb_qcnt: array<u32>;
@group(1) @binding(9) var<storage, read> lm_nb_vscan: array<u32>;
@group(1) @binding(10) var<storage, read> lm_nb_qscan: array<u32>;
@group(1) @binding(11) var<storage, read_write> lm_nb_verts: array<vec4f>;
@group(1) @binding(12) var<storage, read_write> lm_nb_norms: array<vec4f>;
@group(1) @binding(13) var<storage, read_write> lm_nb_indices: array<u32>;

const LM_NB_B: u32 = 4u;
const LM_NB_PTS: u32 = 5u;
const LM_NB_CELLS: u32 = 64u;
const LM_NB_PTS3: u32 = 125u;
const LM_NB_NONE: u32 = 0xffffffffu;

// THE one world-coordinate expression for every lattice-point evaluation in
// this module. It depends only on the GLOBAL point index — never on which
// block asked — so duplicated evaluations of a shared point are bit-identical
// (the module docs' boundary-consistency argument).
fn lm_nb_point(g: vec3u) -> vec3f {
	return lm_np.origin.xyz + vec3f(f32(g.x), f32(g.y), f32(g.z)) * lm_np.origin.w;
}

// kernel_core::sdf::central_difference, eps 1e-4 — identical to the dense
// extractor's lm_grad.
fn lm_nb_grad(p: vec3f) -> vec3f {
	let e = 1e-4;
	let dx = lm_field(p + vec3f(e, 0.0, 0.0)) - lm_field(p - vec3f(e, 0.0, 0.0));
	let dy = lm_field(p + vec3f(0.0, e, 0.0)) - lm_field(p - vec3f(0.0, e, 0.0));
	let dz = lm_field(p + vec3f(0.0, 0.0, e)) - lm_field(p - vec3f(0.0, 0.0, e));
	let g = vec3f(dx, dy, dz);
	let len = length(g);
	if (len > 1e-12) {
		return g / len;
	}
	return vec3f(0.0, 0.0, 1.0);
}

// Linear thread id for one-thread-per-block passes on a 2D-linearized grid
// (same wid.x + wid.y * nwg.x scheme as the scan kernels).
fn lm_nb_linear(wid: vec3u, nwg: vec3u, lid: vec3u) -> u32 {
	return (wid.x + wid.y * nwg.x) * 256u + lid.x;
}

// COARSE PASS: one thread per block. `reach` is the exact distance from the
// block's integer centre lattice point to the farthest corner of the block's
// point box (partial edge blocks get their true, smaller reach — no clamping
// slop). Flag active iff |d(centre)| <= band + reach (module docs, Lipschitz).
@compute @workgroup_size(256)
fn lm_nb_coarse(@builtin(workgroup_id) wid: vec3u, @builtin(num_workgroups) nwg: vec3u, @builtin(local_invocation_id) lid: vec3u) {
	let bi = lm_nb_linear(wid, nwg, lid);
	if (bi >= lm_np.bdims.w) {
		return;
	}
	let bx = bi % lm_np.bdims.x;
	let bt = bi / lm_np.bdims.x;
	let b = vec3u(bx, bt % lm_np.bdims.y, bt / lm_np.bdims.y);
	let c0 = b * LM_NB_B;
	let c1 = min(c0 + vec3u(LM_NB_B), lm_np.cdims.xyz);
	let cc = (c0 + c1) / 2u;
	let r = max(vec3f(cc - c0), vec3f(c1 - cc));
	let reach = length(r) * lm_np.origin.w;
	let d = lm_field(lm_nb_point(cc));
	lm_nb_bflag[bi] = select(0u, 1u, abs(d) <= lm_np.band.x + reach);
}

// COMPACT scatter (runs after the flag scan): dense active-block list plus
// the inverse block -> compact-slot map used for cross-block neighbour reads.
@compute @workgroup_size(256)
fn lm_nb_scatter(@builtin(workgroup_id) wid: vec3u, @builtin(num_workgroups) nwg: vec3u, @builtin(local_invocation_id) lid: vec3u) {
	let bi = lm_nb_linear(wid, nwg, lid);
	if (bi >= lm_np.bdims.w) {
		return;
	}
	if (lm_nb_bflag[bi] == 1u) {
		let s = lm_nb_bscan[bi];
		lm_nb_active[s] = bi;
		lm_nb_bslot[bi] = s;
	} else {
		lm_nb_bslot[bi] = LM_NB_NONE;
	}
}

// Block coordinates of active slot w.
fn lm_nb_block_of(w: u32) -> vec3u {
	let bi = lm_nb_active[w];
	let bx = bi % lm_np.bdims.x;
	let bt = bi / lm_np.bdims.x;
	return vec3u(bx, bt % lm_np.bdims.y, bt / lm_np.bdims.y);
}

// REFINE 1: per-active-block corner sampling. One workgroup per active block,
// one thread per (B+1)^3 corner point. The single lm_field call site below,
// on the block-independent lm_nb_point coordinate, IS the bit-identical
// duplicate-evaluation guarantee the closure argument rests on.
@compute @workgroup_size(5, 5, 5)
fn lm_nb_corner_pass(@builtin(workgroup_id) wid: vec3u, @builtin(num_workgroups) nwg: vec3u, @builtin(local_invocation_id) lid: vec3u) {
	let w = wid.x + wid.y * nwg.x;
	if (w >= lm_np.cdims.w) {
		return;
	}
	let g = lm_nb_block_of(w) * LM_NB_B + lid;
	if (g.x > lm_np.cdims.x || g.y > lm_np.cdims.y || g.z > lm_np.cdims.z) {
		return;
	}
	let lp = lid.x + LM_NB_PTS * (lid.y + LM_NB_PTS * lid.z);
	lm_nb_corners[w * LM_NB_PTS3 + lp] = lm_field(lm_nb_point(g));
}

// Cell-local Surface Nets vertex from the block-local corner grid: xyz =
// average crossing (cell-local 0..1), w = crossing count. Mirrors the dense
// lm_cell_point — the spliced edge unroll is generated by the SAME function.
fn lm_nb_cell_point(w: u32, l: vec3u) -> vec4f {
	var grid: array<f32, 8>;
	var mask = 0u;
	for (var g = 0u; g < 8u; g++) {
		let oi = g & 1u;
		let oj = (g >> 1u) & 1u;
		let ok = (g >> 2u) & 1u;
		let val = lm_nb_corners[w * LM_NB_PTS3 + (l.x + oi) + LM_NB_PTS * ((l.y + oj) + LM_NB_PTS * (l.z + ok))];
		grid[g] = val;
		if (val < 0.0) {
			mask = mask | (1u << g);
		}
	}
	if (mask == 0u || mask == 255u) {
		return vec4f(0.0);
	}
	var v = vec3f(0.0);
	var ecount = 0.0;
//__EDGE_UNROLL__
	if (ecount == 0.0) {
		return vec4f(0.0);
	}
	return vec4f(v / ecount, ecount);
}

// Corner mask of the cell at local coords l of active block w.
fn lm_nb_cell_mask(w: u32, l: vec3u) -> u32 {
	var mask = 0u;
	for (var g = 0u; g < 8u; g++) {
		let oi = g & 1u;
		let oj = (g >> 1u) & 1u;
		let ok = (g >> 2u) & 1u;
		let val = lm_nb_corners[w * LM_NB_PTS3 + (l.x + oi) + LM_NB_PTS * ((l.y + oj) + LM_NB_PTS * (l.z + ok))];
		if (val < 0.0) {
			mask = mask | (1u << g);
		}
	}
	return mask;
}

// REFINE 2: per-cell mask + vertex flag. maskflag = (corner mask << 1) |
// has-vertex (single source for the later passes); the separate pure-0/1
// lm_nb_vflag copy is the prefix-sum input.
@compute @workgroup_size(4, 4, 4)
fn lm_nb_mask_pass(@builtin(workgroup_id) wid: vec3u, @builtin(num_workgroups) nwg: vec3u, @builtin(local_invocation_id) lid: vec3u) {
	let w = wid.x + wid.y * nwg.x;
	if (w >= lm_np.cdims.w) {
		return;
	}
	let slot = w * LM_NB_CELLS + lid.x + LM_NB_B * (lid.y + LM_NB_B * lid.z);
	let c = lm_nb_block_of(w) * LM_NB_B + lid;
	if (c.x >= lm_np.cdims.x || c.y >= lm_np.cdims.y || c.z >= lm_np.cdims.z) {
		lm_nb_maskflag[slot] = 0u;
		lm_nb_vflag[slot] = 0u;
		return;
	}
	let has_vertex = select(0u, 1u, lm_nb_cell_point(w, lid).w > 0.0);
	lm_nb_maskflag[slot] = (lm_nb_cell_mask(w, lid) << 1u) | has_vertex;
	lm_nb_vflag[slot] = has_vertex;
}

// Compact-cell slot of the global cell c, or LM_NB_NONE when c's block was
// not refined. Caller guarantees c is in range.
fn lm_nb_slot_at(c: vec3u) -> u32 {
	let b = c / LM_NB_B;
	let bi = b.x + lm_np.bdims.x * (b.y + lm_np.bdims.y * b.z);
	let s = lm_nb_bslot[bi];
	if (s == LM_NB_NONE) {
		return LM_NB_NONE;
	}
	let l = c - b * LM_NB_B;
	return s * LM_NB_CELLS + l.x + LM_NB_B * (l.y + LM_NB_B * l.z);
}

// 1 iff global cell c holds a Surface Nets vertex. Cells of unrefined blocks
// read 0 — by the Lipschitz flag test that never applies to a cell incident
// to a straddling edge, so this mirrors the dense lm_vflag lookup exactly.
fn lm_nb_flag_at(c: vec3u) -> u32 {
	let s = lm_nb_slot_at(c);
	if (s == LM_NB_NONE) {
		return 0u;
	}
	return lm_nb_maskflag[s] & 1u;
}

// Vertex id of the global cell c; only called after lm_nb_flag_at(c) == 1.
fn lm_nb_vid_at(c: vec3u) -> u32 {
	return lm_nb_vscan[lm_nb_slot_at(c)];
}

// REFINE 3: quad counts — the dense lm_qcnt_pass conditions verbatim
// (straddling minimal edge per axis, in-range on the two perpendicular axes,
// all three neighbour cells holding vertices), with neighbours looked up
// through the block slot map.
@compute @workgroup_size(4, 4, 4)
fn lm_nb_qcnt_pass(@builtin(workgroup_id) wid: vec3u, @builtin(num_workgroups) nwg: vec3u, @builtin(local_invocation_id) lid: vec3u) {
	let w = wid.x + wid.y * nwg.x;
	if (w >= lm_np.cdims.w) {
		return;
	}
	let slot = w * LM_NB_CELLS + lid.x + LM_NB_B * (lid.y + LM_NB_B * lid.z);
	let mf = lm_nb_maskflag[slot];
	var cnt = 0u;
	if ((mf & 1u) == 1u) {
		let c = lm_nb_block_of(w) * LM_NB_B + lid;
		let mask = mf >> 1u;
		// axis 0: minimal edge (0,1); neighbours at -y, -y-z, -z
		if (((mask ^ (mask >> 1u)) & 1u) != 0u && c.y > 0u && c.z > 0u) {
			if (lm_nb_flag_at(c - vec3u(0u, 1u, 0u)) == 1u && lm_nb_flag_at(c - vec3u(0u, 1u, 1u)) == 1u && lm_nb_flag_at(c - vec3u(0u, 0u, 1u)) == 1u) {
				cnt = cnt + 1u;
			}
		}
		// axis 1: minimal edge (0,2); neighbours at -z, -z-x, -x
		if (((mask ^ (mask >> 2u)) & 1u) != 0u && c.z > 0u && c.x > 0u) {
			if (lm_nb_flag_at(c - vec3u(0u, 0u, 1u)) == 1u && lm_nb_flag_at(c - vec3u(1u, 0u, 1u)) == 1u && lm_nb_flag_at(c - vec3u(1u, 0u, 0u)) == 1u) {
				cnt = cnt + 1u;
			}
		}
		// axis 2: minimal edge (0,4); neighbours at -x, -x-y, -y
		if (((mask ^ (mask >> 4u)) & 1u) != 0u && c.x > 0u && c.y > 0u) {
			if (lm_nb_flag_at(c - vec3u(1u, 0u, 0u)) == 1u && lm_nb_flag_at(c - vec3u(1u, 1u, 0u)) == 1u && lm_nb_flag_at(c - vec3u(0u, 1u, 0u)) == 1u) {
				cnt = cnt + 1u;
			}
		}
	}
	lm_nb_qcnt[slot] = cnt;
}

// REFINE 4: compacted vertex + normal emission (dense lm_vert_pass, with the
// cell's world coords reconstructed from block + local).
@compute @workgroup_size(4, 4, 4)
fn lm_nb_vert_pass(@builtin(workgroup_id) wid: vec3u, @builtin(num_workgroups) nwg: vec3u, @builtin(local_invocation_id) lid: vec3u) {
	let w = wid.x + wid.y * nwg.x;
	if (w >= lm_np.cdims.w) {
		return;
	}
	let slot = w * LM_NB_CELLS + lid.x + LM_NB_B * (lid.y + LM_NB_B * lid.z);
	if ((lm_nb_maskflag[slot] & 1u) == 0u) {
		return;
	}
	let c = lm_nb_block_of(w) * LM_NB_B + lid;
	let r = lm_nb_cell_point(w, lid);
	let world = lm_np.origin.xyz + (vec3f(f32(c.x), f32(c.y), f32(c.z)) + r.xyz) * lm_np.origin.w;
	let vid = lm_nb_vscan[slot];
	lm_nb_verts[vid] = vec4f(world, 1.0);
	lm_nb_norms[vid] = vec4f(lm_nb_grad(world), 0.0);
}

// Identical to the dense lm_emit_quad (same diagonal split, same orientation
// rule from the corner-0 sign).
fn lm_nb_emit_quad(slot_base: u32, inside0: bool, q0: u32, q1: u32, q2: u32, q3: u32) {
	let s = slot_base * 6u;
	if (inside0) {
		lm_nb_indices[s] = q0;
		lm_nb_indices[s + 1u] = q1;
		lm_nb_indices[s + 2u] = q2;
		lm_nb_indices[s + 3u] = q0;
		lm_nb_indices[s + 4u] = q2;
		lm_nb_indices[s + 5u] = q3;
	} else {
		lm_nb_indices[s] = q0;
		lm_nb_indices[s + 1u] = q3;
		lm_nb_indices[s + 2u] = q2;
		lm_nb_indices[s + 3u] = q0;
		lm_nb_indices[s + 4u] = q2;
		lm_nb_indices[s + 5u] = q1;
	}
}

// REFINE 5: quad emission — the dense lm_quad_pass with slot-map neighbour
// lookups; conditions match lm_nb_qcnt_pass exactly so every counted slot is
// filled (deterministic prefix-sum slot assignment, no atomics).
@compute @workgroup_size(4, 4, 4)
fn lm_nb_quad_pass(@builtin(workgroup_id) wid: vec3u, @builtin(num_workgroups) nwg: vec3u, @builtin(local_invocation_id) lid: vec3u) {
	let w = wid.x + wid.y * nwg.x;
	if (w >= lm_np.cdims.w) {
		return;
	}
	let slot = w * LM_NB_CELLS + lid.x + LM_NB_B * (lid.y + LM_NB_B * lid.z);
	let mf = lm_nb_maskflag[slot];
	if ((mf & 1u) == 0u) {
		return;
	}
	let c = lm_nb_block_of(w) * LM_NB_B + lid;
	let mask = mf >> 1u;
	let inside0 = (mask & 1u) != 0u;
	var emitted = 0u;
	if (((mask ^ (mask >> 1u)) & 1u) != 0u && c.y > 0u && c.z > 0u) {
		let n1 = c - vec3u(0u, 1u, 0u);
		let n2 = c - vec3u(0u, 1u, 1u);
		let n3 = c - vec3u(0u, 0u, 1u);
		if (lm_nb_flag_at(n1) == 1u && lm_nb_flag_at(n2) == 1u && lm_nb_flag_at(n3) == 1u) {
			lm_nb_emit_quad(lm_nb_qscan[slot] + emitted, inside0, lm_nb_vscan[slot], lm_nb_vid_at(n1), lm_nb_vid_at(n2), lm_nb_vid_at(n3));
			emitted = emitted + 1u;
		}
	}
	if (((mask ^ (mask >> 2u)) & 1u) != 0u && c.z > 0u && c.x > 0u) {
		let n1 = c - vec3u(0u, 0u, 1u);
		let n2 = c - vec3u(1u, 0u, 1u);
		let n3 = c - vec3u(1u, 0u, 0u);
		if (lm_nb_flag_at(n1) == 1u && lm_nb_flag_at(n2) == 1u && lm_nb_flag_at(n3) == 1u) {
			lm_nb_emit_quad(lm_nb_qscan[slot] + emitted, inside0, lm_nb_vscan[slot], lm_nb_vid_at(n1), lm_nb_vid_at(n2), lm_nb_vid_at(n3));
			emitted = emitted + 1u;
		}
	}
	if (((mask ^ (mask >> 4u)) & 1u) != 0u && c.x > 0u && c.y > 0u) {
		let n1 = c - vec3u(1u, 0u, 0u);
		let n2 = c - vec3u(1u, 1u, 0u);
		let n3 = c - vec3u(0u, 1u, 0u);
		if (lm_nb_flag_at(n1) == 1u && lm_nb_flag_at(n2) == 1u && lm_nb_flag_at(n3) == 1u) {
			lm_nb_emit_quad(lm_nb_qscan[slot] + emitted, inside0, lm_nb_vscan[slot], lm_nb_vid_at(n1), lm_nb_vid_at(n2), lm_nb_vid_at(n3));
			emitted = emitted + 1u;
		}
	}
}
"#;

/// A compiled GPU narrow-band Surface Nets extractor for one tree (compile
/// once, extract at any domain/resolution/band).
pub struct GpuNarrowBand {
	device: wgpu::Device,
	queue: wgpu::Queue,
	limits: wgpu::Limits,
	scene_bind: wgpu::BindGroup,
	layouts: [wgpu::BindGroupLayout; 7],
	pipelines: [wgpu::ComputePipeline; 7],
	scan: GpuScan,
}

impl GpuNarrowBand {
	/// Lower `tree` and compile the seven narrow-band pipelines + the scan.
	pub fn compile(ctx: &GpuContext, tree: &GpuNode) -> Result<GpuNarrowBand, GpuError> {
		let lowered = lower(tree)?;
		let wgsl = format!("{}{}", lowered.wgsl, NB_TEMPLATE.replace("//__EDGE_UNROLL__", &edge_unroll()));
		let (scene_layout, scene_bind) = ctx.scene_resources(&lowered);
		let mk = |entries: &[wgpu::BindGroupLayoutEntry]| {
			ctx.device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor { label: Some("lm-nb"), entries })
		};
		let uniform = GpuContext::uniform_entry;
		let storage = GpuContext::storage_entry;
		// Per-pass layouts keep every pipeline at <= 8 storage buffers per stage
		// (the wgpu default): scene(2) + at most 6 pass buffers. `read_only`
		// mirrors each buffer's WGSL declaration (bscan/vscan/qscan are the only
		// `var<storage, read>` bindings).
		let layouts = [
			// coarse: writes block flags
			mk(&[uniform(0), storage(1, false)]),
			// scatter: reads flags + scan, writes active list + slot map
			mk(&[uniform(0), storage(1, false), storage(2, true), storage(3, false), storage(4, false)]),
			// corners: reads active list, writes per-block corner samples
			mk(&[uniform(0), storage(4, false), storage(5, false)]),
			// mask: reads active + corners, writes maskflag + scan input
			mk(&[uniform(0), storage(4, false), storage(5, false), storage(6, false), storage(7, false)]),
			// qcnt: reads slot map + active + maskflag, writes quad counts
			mk(&[uniform(0), storage(3, false), storage(4, false), storage(6, false), storage(8, false)]),
			// verts: reads active/corners/maskflag/vscan, writes verts + norms
			mk(&[uniform(0), storage(4, false), storage(5, false), storage(6, false), storage(9, true), storage(11, false), storage(12, false)]),
			// quads: reads slot/active/maskflag/vscan/qscan, writes indices
			mk(&[uniform(0), storage(3, false), storage(4, false), storage(6, false), storage(9, true), storage(10, true), storage(13, false)]),
		];
		let names = [
			"lm_nb_coarse",
			"lm_nb_scatter",
			"lm_nb_corner_pass",
			"lm_nb_mask_pass",
			"lm_nb_qcnt_pass",
			"lm_nb_vert_pass",
			"lm_nb_quad_pass",
		];
		let entries: Vec<(&str, Vec<&wgpu::BindGroupLayout>)> =
			names.iter().zip(layouts.iter()).map(|(name, layout)| (*name, vec![&scene_layout, layout])).collect();
		let entry_refs: Vec<(&str, &[&wgpu::BindGroupLayout])> = entries.iter().map(|(n, l)| (*n, l.as_slice())).collect();
		let pipelines = ctx.compile_pipelines(&wgsl, &entry_refs)?;
		let pipelines: [wgpu::ComputePipeline; 7] = pipelines.try_into().unwrap_or_else(|_| panic!("seven pipelines requested"));
		Ok(GpuNarrowBand {
			device: ctx.device.clone(),
			queue: ctx.queue.clone(),
			limits: ctx.device.limits(),
			scene_bind,
			layouts,
			pipelines,
			scan: GpuScan::new(ctx)?,
		})
	}

	fn storage(&self, label: &str, bytes: u64) -> wgpu::Buffer {
		self.device.create_buffer(&wgpu::BufferDescriptor {
			label: Some(label),
			size: bytes.max(4),
			usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
			mapped_at_creation: false,
		})
	}

	fn staging(&self, label: &str, bytes: u64) -> wgpu::Buffer {
		self.device.create_buffer(&wgpu::BufferDescriptor {
			label: Some(label),
			size: bytes,
			usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
			mapped_at_creation: false,
		})
	}

	fn check_size(&self, what: &'static str, bytes: u64) -> Result<(), GpuError> {
		let limit = (self.limits.max_storage_buffer_binding_size as u64).min(self.limits.max_buffer_size);
		if bytes > limit {
			return Err(GpuError::TooLarge { what, needed_bytes: bytes, limit_bytes: limit });
		}
		Ok(())
	}

	/// `(x, y)` workgroup dispatch covering `n` workgroups (65535 per-dim cap;
	/// the shaders linearize as `wid.x + wid.y * num_workgroups.x`).
	fn dispatch_2d(n: u32) -> (u32, u32) {
		let x = n.clamp(1, 65_535);
		(x, n.div_ceil(x).max(1))
	}

	/// Extract the iso-surface over `domain` at `resolution`, refining only
	/// blocks within `band` (world units, clamped up to the documented 2-voxel
	/// floor — see module docs) of the surface.
	///
	/// Same lattice and guards as [`crate::GpuSurfaceNets::extract`]:
	/// unmeshable domains yield an EMPTY mesh; sizes beyond the narrow-band
	/// caps or device buffer limits refuse loudly with [`GpuError::TooLarge`].
	/// Unlike the dense path, domains beyond the dense 2²⁸-cell cap are
	/// accepted — block-grid memory is 1/64th of the cell count and everything
	/// else scales with the active band.
	pub fn extract(&self, domain: Aabb, resolution: impl Into<Resolution>, band: f32) -> Result<Mesh, GpuError> {
		self.extract_with_stats(domain, resolution, band).map(|(mesh, _)| mesh)
	}

	/// [`GpuNarrowBand::extract`] plus the [`NarrowBandStats`] work receipt.
	pub fn extract_with_stats(&self, domain: Aabb, resolution: impl Into<Resolution>, band: f32) -> Result<(Mesh, NarrowBandStats), GpuError> {
		let vs = resolution.into().voxel_size(domain);
		let size = domain.size();
		if !domain.min.is_finite() || !domain.max.is_finite() || size.min_element() <= 0.0 || !vs.is_finite() || vs <= 0.0 {
			return Ok((Mesh::new(), NarrowBandStats::default()));
		}
		// Same lattice arithmetic as the dense extractor (identical grids), but
		// capped by the narrow-band ceiling instead of the dense one.
		let counts = [(size.x / vs).ceil(), (size.y / vs).ceil(), (size.z / vs).ceil()];
		let cells = (counts[0] as f64 + 3.0) * (counts[1] as f64 + 3.0) * (counts[2] as f64 + 3.0);
		if !(cells.is_finite() && cells <= NB_MAX_LATTICE_CELLS) {
			return Err(GpuError::TooLarge {
				what: "conceptual lattice (narrow-band cap)",
				needed_bytes: if cells.is_finite() { (cells * 4.0) as u64 } else { u64::MAX },
				limit_bytes: (NB_MAX_LATTICE_CELLS * 4.0) as u64,
			});
		}
		if counts.iter().any(|&c| c as f64 + 3.0 > MAX_AXIS_POINTS) {
			return Err(GpuError::TooLarge {
				what: "lattice axis (f32-exact point coordinates)",
				needed_bytes: counts.iter().fold(0u64, |m, &c| m.max(c as u64 + 3)),
				limit_bytes: MAX_AXIS_POINTS as u64,
			});
		}
		let (nx, ny, nz) = (counts[0] as u32 + 3, counts[1] as u32 + 3, counts[2] as u32 + 3);
		let origin = domain.min - Vec3::splat(vs);
		let (cdx, cdy, cdz) = (nx - 1, ny - 1, nz - 1);
		let np = (nx as u64) * (ny as u64) * (nz as u64);
		let (bdx, bdy, bdz) = (cdx.div_ceil(B), cdy.div_ceil(B), cdz.div_ceil(B));
		let total_blocks = (bdx as u64) * (bdy as u64) * (bdz as u64);
		if total_blocks > u32::MAX as u64 {
			return Err(GpuError::TooLarge {
				what: "block grid (u32 block indexing)",
				needed_bytes: total_blocks * 4,
				limit_bytes: (u32::MAX as u64) * 4,
			});
		}
		self.check_size("block flags", total_blocks * 4)?;
		self.check_size("block slot map", total_blocks * 4)?;
		// The DOCUMENTED CLAMP: band below the safe floor (including 0,
		// negative, NaN — f32::max discards a NaN operand) clamps UP to it.
		let band_eff = band.max(MIN_BAND_VOXELS * vs);

		let params = |n_active: u32| {
			let mut bytes = Vec::with_capacity(64);
			bytes.extend_from_slice(bytemuck::cast_slice(&[cdx, cdy, cdz, n_active, bdx, bdy, bdz, total_blocks as u32]));
			bytes.extend_from_slice(bytemuck::cast_slice(&[origin.x, origin.y, origin.z, vs, band_eff, 0.0, 0.0, 0.0]));
			self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
				label: Some("lm-nb-params"),
				contents: &bytes,
				usage: wgpu::BufferUsages::UNIFORM,
			})
		};
		let bind = |params: &wgpu::Buffer, layout: &wgpu::BindGroupLayout, bufs: &[(u32, &wgpu::Buffer)]| {
			let mut entries = vec![wgpu::BindGroupEntry { binding: 0, resource: params.as_entire_binding() }];
			entries.extend(bufs.iter().map(|(b, buf)| wgpu::BindGroupEntry { binding: *b, resource: buf.as_entire_binding() }));
			self.device.create_bind_group(&wgpu::BindGroupDescriptor { label: Some("lm-nb"), layout, entries: &entries })
		};
		let run = |encoder: &mut wgpu::CommandEncoder, pipeline: &wgpu::ComputePipeline, group: &wgpu::BindGroup, d: (u32, u32)| {
			let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor { label: Some("lm-nb"), timestamp_writes: None });
			pass.set_pipeline(pipeline);
			pass.set_bind_group(0, &self.scene_bind, &[]);
			pass.set_bind_group(1, group, &[]);
			pass.dispatch_workgroups(d.0, d.1, 1);
		};

		// COARSE: flag surface-possible blocks.
		let bflag = self.storage("lm-nb-bflag", total_blocks * 4);
		let p_coarse = params(0);
		let block_threads = Self::dispatch_2d((total_blocks as u32).div_ceil(256));
		let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("lm-nb-coarse") });
		run(&mut encoder, &self.pipelines[0], &bind(&p_coarse, &self.layouts[0], &[(1, &bflag)]), block_threads);
		self.queue.submit(Some(encoder.finish()));

		// COMPACT: scan the flags into slots; empty band => empty mesh (a field
		// with no straddling cell anywhere flags no block — mirrors the dense
		// A − A / bare-half-space empty results).
		let (bscan, n_active) = self.scan.exclusive_scan(&bflag, total_blocks as u32);
		let mut stats = NarrowBandStats {
			active_blocks: n_active as u64,
			total_blocks,
			samples_evaluated: total_blocks,
			dense_samples: np,
		};
		if n_active == 0 {
			return Ok((Mesh::new(), stats));
		}
		let n_active_cells = (n_active as u64) * B_CELLS;
		if n_active_cells > u32::MAX as u64 {
			return Err(GpuError::TooLarge {
				what: "active cells (scan index range)",
				needed_bytes: n_active_cells * 4,
				limit_bytes: (u32::MAX as u64) * 4,
			});
		}
		self.check_size("active-block corner samples", (n_active as u64) * B_PTS * 4)?;
		self.check_size("active-cell flags", n_active_cells * 4)?;

		let bslot = self.storage("lm-nb-bslot", total_blocks * 4);
		let active = self.storage("lm-nb-active", (n_active as u64) * 4);
		let corners = self.storage("lm-nb-corners", (n_active as u64) * B_PTS * 4);
		let maskflag = self.storage("lm-nb-maskflag", n_active_cells * 4);
		let vflag = self.storage("lm-nb-vflag", n_active_cells * 4);
		let qcnt = self.storage("lm-nb-qcnt", n_active_cells * 4);
		let active_rb = self.staging("lm-nb-active-rb", (n_active as u64) * 4);
		let p = params(n_active);
		let per_block = Self::dispatch_2d(n_active);

		// REFINE part A: scatter + corner sampling + cell masks/flags/counts.
		let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("lm-nb-refine-a") });
		run(
			&mut encoder,
			&self.pipelines[1],
			&bind(&p, &self.layouts[1], &[(1, &bflag), (2, &bscan), (3, &bslot), (4, &active)]),
			block_threads,
		);
		run(&mut encoder, &self.pipelines[2], &bind(&p, &self.layouts[2], &[(4, &active), (5, &corners)]), per_block);
		run(
			&mut encoder,
			&self.pipelines[3],
			&bind(&p, &self.layouts[3], &[(4, &active), (5, &corners), (6, &maskflag), (7, &vflag)]),
			per_block,
		);
		run(
			&mut encoder,
			&self.pipelines[4],
			&bind(&p, &self.layouts[4], &[(3, &bslot), (4, &active), (6, &maskflag), (8, &qcnt)]),
			per_block,
		);
		encoder.copy_buffer_to_buffer(&active, 0, &active_rb, 0, (n_active as u64) * 4);
		self.queue.submit(Some(encoder.finish()));

		// Prefix sums: compacted vertex ids and quad slots (+ totals).
		let (vscan, nv) = self.scan.exclusive_scan(&vflag, n_active_cells as u32);
		let (qscan, nq) = self.scan.exclusive_scan(&qcnt, n_active_cells as u32);

		// The honest sample count: coarse centres + each active block's
		// in-lattice corner points (partial edge blocks evaluate fewer — the
		// corner pass early-returns outside the lattice, and this sum mirrors
		// that exactly).
		let active_bytes = crate::map_read(&self.device, &active_rb);
		let abids: &[u32] = bytemuck::cast_slice(&active_bytes);
		for &bi in abids {
			let bx = bi % bdx;
			let bt = bi / bdx;
			let (by, bz) = (bt % bdy, bt / bdy);
			let px = (cdx - bx * B).min(B) as u64 + 1;
			let py = (cdy - by * B).min(B) as u64 + 1;
			let pz = (cdz - bz * B).min(B) as u64 + 1;
			stats.samples_evaluated += px * py * pz;
		}
		if nv == 0 {
			return Ok((Mesh::new(), stats));
		}
		self.check_size("vertex buffer", nv as u64 * 16)?;
		self.check_size("index buffer", nq.max(1) as u64 * 24)?;

		// REFINE part B: emit compacted vertices/normals and quad indices.
		let verts = self.storage("lm-nb-verts", nv as u64 * 16);
		let norms = self.storage("lm-nb-norms", nv as u64 * 16);
		let indices = self.storage("lm-nb-indices", nq.max(1) as u64 * 24);
		let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("lm-nb-refine-b") });
		run(
			&mut encoder,
			&self.pipelines[5],
			&bind(&p, &self.layouts[5], &[(4, &active), (5, &corners), (6, &maskflag), (9, &vscan), (11, &verts), (12, &norms)]),
			per_block,
		);
		if nq > 0 {
			run(
				&mut encoder,
				&self.pipelines[6],
				&bind(&p, &self.layouts[6], &[(3, &bslot), (4, &active), (6, &maskflag), (9, &vscan), (10, &qscan), (13, &indices)]),
				per_block,
			);
		}
		let verts_rb = self.staging("lm-nb-verts-rb", nv as u64 * 16);
		let norms_rb = self.staging("lm-nb-norms-rb", nv as u64 * 16);
		let idx_rb = self.staging("lm-nb-idx-rb", nq.max(1) as u64 * 24);
		encoder.copy_buffer_to_buffer(&verts, 0, &verts_rb, 0, nv as u64 * 16);
		encoder.copy_buffer_to_buffer(&norms, 0, &norms_rb, 0, nv as u64 * 16);
		if nq > 0 {
			encoder.copy_buffer_to_buffer(&indices, 0, &idx_rb, 0, nq as u64 * 24);
		}
		self.queue.submit(Some(encoder.finish()));

		// Readback and mesh assembly — the same finishing step as the dense
		// extractor (outward orientation via signed volume).
		let vbytes = crate::map_read(&self.device, &verts_rb);
		let nbytes = crate::map_read(&self.device, &norms_rb);
		let vf: &[f32] = bytemuck::cast_slice(&vbytes);
		let nf: &[f32] = bytemuck::cast_slice(&nbytes);
		let mut mesh = Mesh::new();
		for i in 0..nv as usize {
			mesh.push_vertex(Vec3::new(vf[4 * i], vf[4 * i + 1], vf[4 * i + 2]));
			mesh.normals.push(Vec3::new(nf[4 * i], nf[4 * i + 1], nf[4 * i + 2]));
		}
		if nq > 0 {
			let ibytes = crate::map_read(&self.device, &idx_rb);
			let idx: &[u32] = bytemuck::cast_slice(&ibytes);
			for t in idx.chunks_exact(3) {
				mesh.push_triangle(t[0], t[1], t[2]);
			}
		}
		mesh.ensure_outward();
		Ok((mesh, stats))
	}
}

/// One-shot convenience mirroring [`crate::gpu_surface_nets`]: compile `tree`
/// and extract over `domain` refining only the narrow band. `band` is in
/// world units and clamps up to the documented 2-voxel floor; pass
/// `Resolution::VoxelSize(vs)` (or a bare `f32` voxel size) as `resolution`.
pub fn extract_narrow_band(
	ctx: &GpuContext,
	tree: &GpuNode,
	domain: Aabb,
	resolution: impl Into<Resolution>,
	band: f32,
) -> Result<Mesh, GpuError> {
	GpuNarrowBand::compile(ctx, tree)?.extract(domain, resolution, band)
}

/// [`extract_narrow_band`] plus the [`NarrowBandStats`] work receipt.
pub fn extract_narrow_band_with_stats(
	ctx: &GpuContext,
	tree: &GpuNode,
	domain: Aabb,
	resolution: impl Into<Resolution>,
	band: f32,
) -> Result<(Mesh, NarrowBandStats), GpuError> {
	GpuNarrowBand::compile(ctx, tree)?.extract_with_stats(domain, resolution, band)
}
