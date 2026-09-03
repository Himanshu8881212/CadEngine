// Copyright (c) LMCAD. Licensed under the MIT License.

//! GPU Surface Nets extraction: dense corner sampling + vertex/quad emission as
//! compute passes with prefix-sum compaction.
//!
//! This mirrors `kernel_core::mesher::surface_nets` step for step — the SAME
//! lattice layout (one padding cell on every side, `ceil(size/vs) + 3` sample
//! points per axis), the SAME cell topology (cube-edge tables generated from
//! `kernel_core::marching::edge_tables`, spliced into the WGSL as an unrolled
//! loop in the same edge order), the SAME vertex placement (average of edge
//! zero-crossings), the SAME quad emission and orientation rules — so its
//! output is directly comparable to the CPU mesher's. Differences are bounded
//! by the GPU field tolerance: vertices shift within ~1e-4/|∇d| and a corner
//! sample within the tolerance of zero may classify differently, changing a
//! few marginal cells. Closure still holds (every straddling minimal edge of
//! the shared corner buffer gets its quad), so the result is a closed surface
//! by the same argument as the CPU mesher's.
//!
//! **Honest role statement**: this is the *preview/bulk* extraction path. The
//! watertight authority for production output remains the CPU's Manifold Dual
//! Contouring (`kernel_implicit::manifold_dual_contour`) — same as the CPU
//! Surface Nets, the one-vertex-per-cell dual can emit non-manifold edges on
//! sub-voxel features, and `check_mesh` is the gate either way.
//!
//! Determinism: vertex ids and triangle slots come from exclusive prefix sums
//! over the cell lattice (no atomics anywhere), so extraction is bit-stable
//! run to run on the same device/driver — pinned by an in-tree test.

use kernel_core::math::{Aabb, Vec3};
use kernel_core::mesh::Mesh;
use kernel_core::mesher::{Resolution, MAX_LATTICE_CELLS};
use wgpu::util::DeviceExt;

use crate::codegen::lower;
use crate::scan::GpuScan;
use crate::tree::GpuNode;
use crate::{GpuContext, GpuError};

/// Generate the unrolled 12-edge crossing accumulation from the SAME tables
/// the CPU meshers use (`kernel_core::marching::edge_tables`) — identical edge
/// order, identical per-edge arithmetic, so vertex placement mirrors the CPU's
/// Phase A exactly. `pub(crate)` so the narrow-band extractor splices the
/// IDENTICAL unroll into its own cell-point function (same topology rules by
/// construction, not by parallel maintenance).
pub(crate) fn edge_unroll() -> String {
	let (cube_edges, _) = kernel_core::marching::edge_tables();
	let mut s = String::new();
	for e in 0..12 {
		let c0 = cube_edges[2 * e];
		let c1 = cube_edges[2 * e + 1];
		let axis = (c0 ^ c1).trailing_zeros() as usize; // c1 = c0 | (1 << axis)
		// Contribution lanes: `t` on the edge axis; c0's corner bit elsewhere
		// (CPU: `if a != b { v[axis] += t-or-1-t } else if a { v[axis] += 1 }`;
		// c0 < c1 means the edge-axis bit of c0 is 0, so the crossing lane is
		// always `t` and the other lanes are c0's bits).
		let lane = |i: usize| -> String {
			if i == axis {
				"t".to_string()
			} else if (c0 >> i) & 1 == 1 {
				"1.0".to_string()
			} else {
				"0.0".to_string()
			}
		};
		s.push_str(&format!(
			"\tif ((((mask >> {c0}u) ^ (mask >> {c1}u)) & 1u) != 0u) {{\n\
			 \t\tlet denom = grid[{c0}] - grid[{c1}];\n\
			 \t\tif (abs(denom) >= 1e-12) {{\n\
			 \t\t\tlet t = grid[{c0}] / denom;\n\
			 \t\t\tecount = ecount + 1.0;\n\
			 \t\t\tv = v + vec3f({}, {}, {});\n\
			 \t\t}}\n\
			 \t}}\n",
			lane(0),
			lane(1),
			lane(2)
		));
	}
	s
}

/// The extraction entry points, appended to a generated field module. The
/// `__EDGE_UNROLL__` placeholder receives [`edge_unroll`]. The three per-axis
/// blocks in the qcnt/quad passes mirror the CPU Phase C loop with
/// `cell_stride = [1, cdx, cdx*cdy]`, `iu = (axis+1)%3`, `iv = (axis+2)%3`.
const EXTRACT_TEMPLATE: &str = r#"
struct LmMcParams {
	dims: vec4u,
	cdims: vec4u,
	origin: vec4f,
}
@group(1) @binding(0) var<uniform> lm_mp: LmMcParams;
@group(1) @binding(1) var<storage, read_write> lm_corners: array<f32>;
@group(1) @binding(2) var<storage, read_write> lm_vflag: array<u32>;
@group(1) @binding(3) var<storage, read_write> lm_qcnt: array<u32>;
@group(1) @binding(4) var<storage, read> lm_vscan: array<u32>;
@group(1) @binding(5) var<storage, read> lm_qscan: array<u32>;
@group(1) @binding(6) var<storage, read_write> lm_verts: array<vec4f>;
@group(1) @binding(7) var<storage, read_write> lm_norms: array<vec4f>;
@group(1) @binding(8) var<storage, read_write> lm_indices: array<u32>;

// kernel_core::sdf::central_difference, eps 1e-4 — the Node tree's CPU
// gradient (Node does not override `gradient`, so this matches the normals
// the CPU mesher samples for composite trees).
fn lm_grad(p: vec3f) -> vec3f {
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

// Corner mask of cell (cx, cy, cz): bit g set when corner g samples inside.
fn lm_cell_mask(cx: u32, cy: u32, cz: u32) -> u32 {
	var mask = 0u;
	for (var g = 0u; g < 8u; g++) {
		let oi = g & 1u;
		let oj = (g >> 1u) & 1u;
		let ok = (g >> 2u) & 1u;
		let val = lm_corners[(cx + oi) + lm_mp.dims.x * ((cy + oj) + lm_mp.dims.y * (cz + ok))];
		if (val < 0.0) {
			mask = mask | (1u << g);
		}
	}
	return mask;
}

// Cell-local Surface Nets vertex: xyz = average crossing (cell-local 0..1),
// w = crossing count (0 when the cell holds no vertex). Mirrors the CPU
// mesher's Phase A.
fn lm_cell_point(cx: u32, cy: u32, cz: u32) -> vec4f {
	var grid: array<f32, 8>;
	var mask = 0u;
	for (var g = 0u; g < 8u; g++) {
		let oi = g & 1u;
		let oj = (g >> 1u) & 1u;
		let ok = (g >> 2u) & 1u;
		let val = lm_corners[(cx + oi) + lm_mp.dims.x * ((cy + oj) + lm_mp.dims.y * (cz + ok))];
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

@compute @workgroup_size(8, 8, 4)
fn lm_corner_pass(@builtin(global_invocation_id) g: vec3u) {
	if (g.x >= lm_mp.dims.x || g.y >= lm_mp.dims.y || g.z >= lm_mp.dims.z) {
		return;
	}
	let p = lm_mp.origin.xyz + vec3f(f32(g.x), f32(g.y), f32(g.z)) * lm_mp.origin.w;
	lm_corners[g.x + lm_mp.dims.x * (g.y + lm_mp.dims.y * g.z)] = lm_field(p);
}

@compute @workgroup_size(8, 8, 4)
fn lm_vflag_pass(@builtin(global_invocation_id) g: vec3u) {
	if (g.x >= lm_mp.cdims.x || g.y >= lm_mp.cdims.y || g.z >= lm_mp.cdims.z) {
		return;
	}
	let ci = g.x + lm_mp.cdims.x * (g.y + lm_mp.cdims.y * g.z);
	lm_vflag[ci] = select(0u, 1u, lm_cell_point(g.x, g.y, g.z).w > 0.0);
}

@compute @workgroup_size(8, 8, 4)
fn lm_qcnt_pass(@builtin(global_invocation_id) g: vec3u) {
	if (g.x >= lm_mp.cdims.x || g.y >= lm_mp.cdims.y || g.z >= lm_mp.cdims.z) {
		return;
	}
	let ci = g.x + lm_mp.cdims.x * (g.y + lm_mp.cdims.y * g.z);
	var cnt = 0u;
	if (lm_vflag[ci] == 1u) {
		let mask = lm_cell_mask(g.x, g.y, g.z);
		// axis 0: minimal edge (0,1); du = cdx, dv = cdx*cdy; needs cy,cz > 0
		if (((mask ^ (mask >> 1u)) & 1u) != 0u && g.y > 0u && g.z > 0u) {
			let du = lm_mp.cdims.x;
			let dv = lm_mp.cdims.x * lm_mp.cdims.y;
			if (lm_vflag[ci - du] == 1u && lm_vflag[ci - du - dv] == 1u && lm_vflag[ci - dv] == 1u) {
				cnt = cnt + 1u;
			}
		}
		// axis 1: minimal edge (0,2); du = cdx*cdy, dv = 1; needs cz,cx > 0
		if (((mask ^ (mask >> 2u)) & 1u) != 0u && g.z > 0u && g.x > 0u) {
			let du = lm_mp.cdims.x * lm_mp.cdims.y;
			let dv = 1u;
			if (lm_vflag[ci - du] == 1u && lm_vflag[ci - du - dv] == 1u && lm_vflag[ci - dv] == 1u) {
				cnt = cnt + 1u;
			}
		}
		// axis 2: minimal edge (0,4); du = 1, dv = cdx; needs cx,cy > 0
		if (((mask ^ (mask >> 4u)) & 1u) != 0u && g.x > 0u && g.y > 0u) {
			let du = 1u;
			let dv = lm_mp.cdims.x;
			if (lm_vflag[ci - du] == 1u && lm_vflag[ci - du - dv] == 1u && lm_vflag[ci - dv] == 1u) {
				cnt = cnt + 1u;
			}
		}
	}
	lm_qcnt[ci] = cnt;
}

@compute @workgroup_size(8, 8, 4)
fn lm_vert_pass(@builtin(global_invocation_id) g: vec3u) {
	if (g.x >= lm_mp.cdims.x || g.y >= lm_mp.cdims.y || g.z >= lm_mp.cdims.z) {
		return;
	}
	let ci = g.x + lm_mp.cdims.x * (g.y + lm_mp.cdims.y * g.z);
	if (lm_vflag[ci] != 1u) {
		return;
	}
	let r = lm_cell_point(g.x, g.y, g.z);
	let world = lm_mp.origin.xyz + (vec3f(f32(g.x), f32(g.y), f32(g.z)) + r.xyz) * lm_mp.origin.w;
	let vid = lm_vscan[ci];
	lm_verts[vid] = vec4f(world, 1.0);
	lm_norms[vid] = vec4f(lm_grad(world), 0.0);
}

fn lm_emit_quad(slot_base: u32, inside0: bool, q0: u32, q1: u32, q2: u32, q3: u32) {
	let s = slot_base * 6u;
	if (inside0) {
		lm_indices[s] = q0;
		lm_indices[s + 1u] = q1;
		lm_indices[s + 2u] = q2;
		lm_indices[s + 3u] = q0;
		lm_indices[s + 4u] = q2;
		lm_indices[s + 5u] = q3;
	} else {
		lm_indices[s] = q0;
		lm_indices[s + 1u] = q3;
		lm_indices[s + 2u] = q2;
		lm_indices[s + 3u] = q0;
		lm_indices[s + 4u] = q2;
		lm_indices[s + 5u] = q1;
	}
}

@compute @workgroup_size(8, 8, 4)
fn lm_quad_pass(@builtin(global_invocation_id) g: vec3u) {
	if (g.x >= lm_mp.cdims.x || g.y >= lm_mp.cdims.y || g.z >= lm_mp.cdims.z) {
		return;
	}
	let ci = g.x + lm_mp.cdims.x * (g.y + lm_mp.cdims.y * g.z);
	if (lm_vflag[ci] != 1u) {
		return;
	}
	let mask = lm_cell_mask(g.x, g.y, g.z);
	let inside0 = (mask & 1u) != 0u;
	var emitted = 0u;
	if (((mask ^ (mask >> 1u)) & 1u) != 0u && g.y > 0u && g.z > 0u) {
		let du = lm_mp.cdims.x;
		let dv = lm_mp.cdims.x * lm_mp.cdims.y;
		if (lm_vflag[ci - du] == 1u && lm_vflag[ci - du - dv] == 1u && lm_vflag[ci - dv] == 1u) {
			lm_emit_quad(lm_qscan[ci] + emitted, inside0, lm_vscan[ci], lm_vscan[ci - du], lm_vscan[ci - du - dv], lm_vscan[ci - dv]);
			emitted = emitted + 1u;
		}
	}
	if (((mask ^ (mask >> 2u)) & 1u) != 0u && g.z > 0u && g.x > 0u) {
		let du = lm_mp.cdims.x * lm_mp.cdims.y;
		let dv = 1u;
		if (lm_vflag[ci - du] == 1u && lm_vflag[ci - du - dv] == 1u && lm_vflag[ci - dv] == 1u) {
			lm_emit_quad(lm_qscan[ci] + emitted, inside0, lm_vscan[ci], lm_vscan[ci - du], lm_vscan[ci - du - dv], lm_vscan[ci - dv]);
			emitted = emitted + 1u;
		}
	}
	if (((mask ^ (mask >> 4u)) & 1u) != 0u && g.x > 0u && g.y > 0u) {
		let du = 1u;
		let dv = lm_mp.cdims.x;
		if (lm_vflag[ci - du] == 1u && lm_vflag[ci - du - dv] == 1u && lm_vflag[ci - dv] == 1u) {
			lm_emit_quad(lm_qscan[ci] + emitted, inside0, lm_vscan[ci], lm_vscan[ci - du], lm_vscan[ci - du - dv], lm_vscan[ci - dv]);
			emitted = emitted + 1u;
		}
	}
}
"#;

/// A compiled GPU Surface Nets extractor for one tree (compile once, extract
/// at any domain/resolution).
pub struct GpuSurfaceNets {
	device: wgpu::Device,
	queue: wgpu::Queue,
	limits: wgpu::Limits,
	scene_bind: wgpu::BindGroup,
	layouts: [wgpu::BindGroupLayout; 5],
	pipelines: [wgpu::ComputePipeline; 5],
	scan: GpuScan,
}

impl GpuSurfaceNets {
	/// Lower `tree` and compile the five extraction pipelines + the scan.
	pub fn compile(ctx: &GpuContext, tree: &GpuNode) -> Result<GpuSurfaceNets, GpuError> {
		let lowered = lower(tree)?;
		let wgsl = format!("{}{}", lowered.wgsl, EXTRACT_TEMPLATE.replace("//__EDGE_UNROLL__", &edge_unroll()));
		let (scene_layout, scene_bind) = ctx.scene_resources(&lowered);
		let mk = |entries: &[wgpu::BindGroupLayoutEntry]| {
			ctx.device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor { label: Some("lm-extract"), entries })
		};
		let uniform = GpuContext::uniform_entry;
		let storage = GpuContext::storage_entry;
		// Per-pass layouts keep every pipeline at <= 8 storage buffers per
		// stage (the wgpu default limit): scene(2) + at most 5 pass buffers.
		let layouts = [
			// corners: writes the corner samples
			mk(&[uniform(0), storage(1, false)]),
			// vflag: reads corners, writes flags
			mk(&[uniform(0), storage(1, false), storage(2, false)]),
			// qcnt: reads corners + flags, writes quad counts
			mk(&[uniform(0), storage(1, false), storage(2, false), storage(3, false)]),
			// verts: reads corners/flags/vscan, writes verts + norms
			mk(&[uniform(0), storage(1, false), storage(2, false), storage(4, true), storage(6, false), storage(7, false)]),
			// quads: reads corners/flags/vscan/qscan, writes indices
			mk(&[uniform(0), storage(1, false), storage(2, false), storage(4, true), storage(5, true), storage(8, false)]),
		];
		let entries: Vec<(&str, Vec<&wgpu::BindGroupLayout>)> = ["lm_corner_pass", "lm_vflag_pass", "lm_qcnt_pass", "lm_vert_pass", "lm_quad_pass"]
			.iter()
			.zip(layouts.iter())
			.map(|(name, layout)| (*name, vec![&scene_layout, layout]))
			.collect();
		let entry_refs: Vec<(&str, &[&wgpu::BindGroupLayout])> = entries.iter().map(|(n, l)| (*n, l.as_slice())).collect();
		let pipelines = ctx.compile_pipelines(&wgsl, &entry_refs)?;
		let mut it = pipelines.into_iter();
		let pipelines = [
			it.next().expect("corner pipeline"),
			it.next().expect("vflag pipeline"),
			it.next().expect("qcnt pipeline"),
			it.next().expect("vert pipeline"),
			it.next().expect("quad pipeline"),
		];
		Ok(GpuSurfaceNets {
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
			size: bytes,
			usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
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

	/// Extract the iso-surface over `domain` at `resolution`.
	///
	/// Mirrors the CPU `surface_nets` guards: unmeshable domains (non-finite,
	/// degenerate, bad voxel size) yield an EMPTY mesh exactly like the CPU
	/// path. Sizes beyond the dense-lattice cap or the device's buffer limits
	/// are refused with a loud [`GpuError::TooLarge`] (the CPU's silent-empty
	/// over-cap behavior is a documented sharp edge this new API does not
	/// inherit).
	pub fn extract(&self, domain: Aabb, resolution: impl Into<Resolution>) -> Result<Mesh, GpuError> {
		let vs = resolution.into().voxel_size(domain);
		let size = domain.size();
		if !domain.min.is_finite() || !domain.max.is_finite() || size.min_element() <= 0.0 || !vs.is_finite() || vs <= 0.0 {
			return Ok(Mesh::new());
		}
		let counts = [(size.x / vs).ceil(), (size.y / vs).ceil(), (size.z / vs).ceil()];
		let cells = (counts[0] as f64 + 3.0) * (counts[1] as f64 + 3.0) * (counts[2] as f64 + 3.0);
		if !(cells.is_finite() && cells <= MAX_LATTICE_CELLS) {
			return Err(GpuError::TooLarge {
				what: "conceptual lattice (dense-mesher cap)",
				needed_bytes: if cells.is_finite() { (cells * 4.0) as u64 } else { u64::MAX },
				limit_bytes: (MAX_LATTICE_CELLS * 4.0) as u64,
			});
		}
		let (nx, ny, nz) = (counts[0] as u32 + 3, counts[1] as u32 + 3, counts[2] as u32 + 3);
		let origin = domain.min - Vec3::splat(vs);
		let (cdx, cdy, cdz) = (nx - 1, ny - 1, nz - 1);
		let np = (nx as u64) * (ny as u64) * (nz as u64);
		let nc = (cdx as u64) * (cdy as u64) * (cdz as u64);
		self.check_size("corner samples", np * 4)?;
		self.check_size("cell flags", nc * 4)?;

		let corners = self.storage("lm-corners", np * 4);
		let vflag = self.storage("lm-vflag", nc * 4);
		let qcnt = self.storage("lm-qcnt", nc * 4);
		let mut params = Vec::with_capacity(48);
		params.extend_from_slice(bytemuck::cast_slice(&[nx, ny, nz, 0, cdx, cdy, cdz, 0]));
		params.extend_from_slice(bytemuck::cast_slice(&[origin.x, origin.y, origin.z, vs]));
		let params = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
			label: Some("lm-extract-params"),
			contents: &params,
			usage: wgpu::BufferUsages::UNIFORM,
		});

		let bind = |layout: &wgpu::BindGroupLayout, bufs: &[(u32, &wgpu::Buffer)]| {
			let mut entries = vec![wgpu::BindGroupEntry { binding: 0, resource: params.as_entire_binding() }];
			entries.extend(bufs.iter().map(|(b, buf)| wgpu::BindGroupEntry { binding: *b, resource: buf.as_entire_binding() }));
			self.device.create_bind_group(&wgpu::BindGroupDescriptor { label: Some("lm-extract"), layout, entries: &entries })
		};
		let pts_dispatch = (nx.div_ceil(8), ny.div_ceil(8), nz.div_ceil(4));
		let cell_dispatch = (cdx.div_ceil(8), cdy.div_ceil(8), cdz.div_ceil(4));
		let run = |encoder: &mut wgpu::CommandEncoder, pipeline: &wgpu::ComputePipeline, group: &wgpu::BindGroup, d: (u32, u32, u32)| {
			let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor { label: Some("lm-extract"), timestamp_writes: None });
			pass.set_pipeline(pipeline);
			pass.set_bind_group(0, &self.scene_bind, &[]);
			pass.set_bind_group(1, group, &[]);
			pass.dispatch_workgroups(d.0, d.1, d.2);
		};

		// Pass 1-3: corner sampling, vertex flags, quad counts.
		let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("lm-extract-a") });
		run(&mut encoder, &self.pipelines[0], &bind(&self.layouts[0], &[(1, &corners)]), pts_dispatch);
		run(&mut encoder, &self.pipelines[1], &bind(&self.layouts[1], &[(1, &corners), (2, &vflag)]), cell_dispatch);
		run(&mut encoder, &self.pipelines[2], &bind(&self.layouts[2], &[(1, &corners), (2, &vflag), (3, &qcnt)]), cell_dispatch);
		self.queue.submit(Some(encoder.finish()));

		// Prefix sums: compacted vertex ids and quad slots (+ totals).
		let (vscan, nv) = self.scan.exclusive_scan(&vflag, nc as u32);
		let (qscan, nq) = self.scan.exclusive_scan(&qcnt, nc as u32);
		if nv == 0 {
			return Ok(Mesh::new());
		}
		self.check_size("vertex buffer", nv as u64 * 16)?;
		self.check_size("index buffer", nq.max(1) as u64 * 24)?;

		// Pass 4-5: emit compacted vertices/normals and quad indices.
		let verts = self.storage("lm-verts", nv as u64 * 16);
		let norms = self.storage("lm-norms", nv as u64 * 16);
		let indices = self.storage("lm-indices", nq.max(1) as u64 * 24);
		let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("lm-extract-b") });
		run(
			&mut encoder,
			&self.pipelines[3],
			&bind(&self.layouts[3], &[(1, &corners), (2, &vflag), (4, &vscan), (6, &verts), (7, &norms)]),
			cell_dispatch,
		);
		if nq > 0 {
			run(
				&mut encoder,
				&self.pipelines[4],
				&bind(&self.layouts[4], &[(1, &corners), (2, &vflag), (4, &vscan), (5, &qscan), (8, &indices)]),
				cell_dispatch,
			);
		}
		let staging = |label: &str, bytes: u64| {
			self.device.create_buffer(&wgpu::BufferDescriptor {
				label: Some(label),
				size: bytes,
				usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
				mapped_at_creation: false,
			})
		};
		let verts_rb = staging("lm-verts-rb", nv as u64 * 16);
		let norms_rb = staging("lm-norms-rb", nv as u64 * 16);
		let idx_rb = staging("lm-idx-rb", nq.max(1) as u64 * 24);
		encoder.copy_buffer_to_buffer(&verts, 0, &verts_rb, 0, nv as u64 * 16);
		encoder.copy_buffer_to_buffer(&norms, 0, &norms_rb, 0, nv as u64 * 16);
		if nq > 0 {
			encoder.copy_buffer_to_buffer(&indices, 0, &idx_rb, 0, nq as u64 * 24);
		}
		self.queue.submit(Some(encoder.finish()));

		// Readback and mesh assembly (same finishing step as the CPU march:
		// outward orientation via signed volume).
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
		Ok(mesh)
	}
}

/// One-shot convenience: compile `tree` and extract over `domain`.
pub fn gpu_surface_nets(ctx: &GpuContext, tree: &GpuNode, domain: Aabb, resolution: impl Into<Resolution>) -> Result<Mesh, GpuError> {
	GpuSurfaceNets::compile(ctx, tree)?.extract(domain, resolution)
}
