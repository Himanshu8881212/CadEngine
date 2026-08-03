// Copyright (c) LMCAD. Licensed under the MIT License.

//! GPU exclusive prefix sum over `u32` buffers — the compaction primitive for
//! the Surface Nets extractor (deterministic slot assignment, no atomics, so
//! extraction output is bit-stable run to run).
//!
//! Classic two-kernel scheme: `scan_block` scans 1024-element blocks (4 per
//! thread serially, then a Hillis–Steele pass over the 256 thread sums in
//! workgroup memory) and emits one sum per block; the block sums are scanned
//! recursively, then `scan_add` folds the scanned block offsets back in. The
//! overall total falls out of the deepest level's single block sum.

use wgpu::util::DeviceExt;

use crate::{GpuContext, GpuError};

const SCAN_WGSL: &str = r#"
struct LmScanParams { n: u32, nblocks: u32, pad0: u32, pad1: u32 }
@group(0) @binding(0) var<uniform> lm_sp: LmScanParams;
@group(0) @binding(1) var<storage, read> lm_src: array<u32>;
@group(0) @binding(2) var<storage, read_write> lm_dst: array<u32>;
@group(0) @binding(3) var<storage, read_write> lm_sums: array<u32>;

var<workgroup> lm_wg: array<u32, 256>;

// Exclusive scan of one 1024-element block (4 serial elements per thread).
@compute @workgroup_size(256)
fn lm_scan_block(
	@builtin(workgroup_id) wid: vec3u,
	@builtin(local_invocation_id) lid: vec3u,
	@builtin(num_workgroups) nwg: vec3u,
) {
	let block = wid.x + wid.y * nwg.x;
	if (block >= lm_sp.nblocks) {
		return;
	}
	let base = block * 1024u + lid.x * 4u;
	var pre: array<u32, 4>;
	var s = 0u;
	for (var e = 0u; e < 4u; e++) {
		let idx = base + e;
		var x = 0u;
		if (idx < lm_sp.n) {
			x = lm_src[idx];
		}
		pre[e] = s;
		s = s + x;
	}
	lm_wg[lid.x] = s;
	workgroupBarrier();
	// Hillis–Steele inclusive scan over the 256 per-thread sums.
	for (var off = 1u; off < 256u; off = off << 1u) {
		var add = 0u;
		if (lid.x >= off) {
			add = lm_wg[lid.x - off];
		}
		workgroupBarrier();
		lm_wg[lid.x] = lm_wg[lid.x] + add;
		workgroupBarrier();
	}
	var prefix = 0u;
	if (lid.x > 0u) {
		prefix = lm_wg[lid.x - 1u];
	}
	for (var e = 0u; e < 4u; e++) {
		let idx = base + e;
		if (idx < lm_sp.n) {
			lm_dst[idx] = prefix + pre[e];
		}
	}
	if (lid.x == 255u) {
		lm_sums[block] = lm_wg[255];
	}
}

// Add the scanned block offsets back into every element of each block.
@compute @workgroup_size(256)
fn lm_scan_add(
	@builtin(workgroup_id) wid: vec3u,
	@builtin(local_invocation_id) lid: vec3u,
	@builtin(num_workgroups) nwg: vec3u,
) {
	let block = wid.x + wid.y * nwg.x;
	if (block >= lm_sp.nblocks) {
		return;
	}
	let add = lm_sums[block];
	let base = block * 1024u + lid.x * 4u;
	for (var e = 0u; e < 4u; e++) {
		let idx = base + e;
		if (idx < lm_sp.n) {
			lm_dst[idx] = lm_dst[idx] + add;
		}
	}
}
"#;

/// Compiled scan pipelines (scene-independent; one per extractor).
pub(crate) struct GpuScan {
	device: wgpu::Device,
	queue: wgpu::Queue,
	layout: wgpu::BindGroupLayout,
	block: wgpu::ComputePipeline,
	add: wgpu::ComputePipeline,
}

/// One scan level: dispatch geometry plus its buffers.
struct Level {
	n: u32,
	nblocks: u32,
	out: wgpu::Buffer,
	sums: wgpu::Buffer,
}

impl GpuScan {
	pub(crate) fn new(ctx: &GpuContext) -> Result<GpuScan, GpuError> {
		let layout = ctx.device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			label: Some("lm-scan"),
			entries: &[
				GpuContext::uniform_entry(0),
				GpuContext::storage_entry(1, true),
				GpuContext::storage_entry(2, false),
				GpuContext::storage_entry(3, false),
			],
		});
		let pipelines = ctx.compile_pipelines(SCAN_WGSL, &[("lm_scan_block", &[&layout]), ("lm_scan_add", &[&layout])])?;
		let mut it = pipelines.into_iter();
		Ok(GpuScan {
			device: ctx.device.clone(),
			queue: ctx.queue.clone(),
			layout,
			block: it.next().expect("scan_block pipeline"),
			add: it.next().expect("scan_add pipeline"),
		})
	}

	fn bind(&self, n: u32, nblocks: u32, src: &wgpu::Buffer, dst: &wgpu::Buffer, sums: &wgpu::Buffer) -> wgpu::BindGroup {
		let params = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
			label: Some("lm-scan-params"),
			contents: bytemuck::cast_slice(&[n, nblocks, 0, 0]),
			usage: wgpu::BufferUsages::UNIFORM,
		});
		self.device.create_bind_group(&wgpu::BindGroupDescriptor {
			label: Some("lm-scan"),
			layout: &self.layout,
			entries: &[
				wgpu::BindGroupEntry { binding: 0, resource: params.as_entire_binding() },
				wgpu::BindGroupEntry { binding: 1, resource: src.as_entire_binding() },
				wgpu::BindGroupEntry { binding: 2, resource: dst.as_entire_binding() },
				wgpu::BindGroupEntry { binding: 3, resource: sums.as_entire_binding() },
			],
		})
	}

	fn storage(&self, label: &str, elems: u64) -> wgpu::Buffer {
		self.device.create_buffer(&wgpu::BufferDescriptor {
			label: Some(label),
			size: elems.max(1) * 4,
			usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
			mapped_at_creation: false,
		})
	}

	/// `(x, y)` workgroup dispatch covering `nblocks` (the per-dimension cap is
	/// 65535, so block ids are linearized as `x + y * num_workgroups.x`).
	fn dispatch_dims(nblocks: u32) -> (u32, u32) {
		let x = nblocks.clamp(1, 65_535);
		(x, nblocks.div_ceil(x).max(1))
	}

	/// Exclusive scan of the `n`-element u32 buffer `src`; returns the scanned
	/// buffer and the grand total. Submits its own command buffers and blocks
	/// for the (8-byte) total readback.
	pub(crate) fn exclusive_scan(&self, src: &wgpu::Buffer, n: u32) -> (wgpu::Buffer, u32) {
		assert!(n > 0, "exclusive_scan: empty input");
		// Build the level chain: scan n elements -> nblocks sums, recurse.
		let mut levels: Vec<Level> = Vec::new();
		let mut len = n;
		loop {
			let nblocks = len.div_ceil(1024);
			levels.push(Level {
				n: len,
				nblocks,
				out: self.storage("lm-scan-out", len as u64),
				sums: self.storage("lm-scan-sums", nblocks as u64),
			});
			if nblocks == 1 {
				break;
			}
			len = nblocks;
		}

		let staging = self.device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("lm-scan-total"),
			size: 4,
			usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
			mapped_at_creation: false,
		});
		let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("lm-scan") });
		// Downsweep: block-scan each level (the input of level i+1 is level i's sums).
		for i in 0..levels.len() {
			let lsrc = if i == 0 { src } else { &levels[i - 1].sums };
			let bind = self.bind(levels[i].n, levels[i].nblocks, lsrc, &levels[i].out, &levels[i].sums);
			let (dx, dy) = Self::dispatch_dims(levels[i].nblocks);
			let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor { label: Some("lm-scan-block"), timestamp_writes: None });
			pass.set_pipeline(&self.block);
			pass.set_bind_group(0, &bind, &[]);
			pass.dispatch_workgroups(dx, dy, 1);
		}
		// Upsweep: add each level's scanned sums back into the level below.
		for i in (0..levels.len().saturating_sub(1)).rev() {
			let lsrc = if i == 0 { src } else { &levels[i - 1].sums };
			// `lm_sums` is bound to the SCANNED sums of this level (the level
			// above's out buffer); src/dst slots mirror the block pass.
			let bind = self.bind(levels[i].n, levels[i].nblocks, lsrc, &levels[i].out, &levels[i + 1].out);
			let (dx, dy) = Self::dispatch_dims(levels[i].nblocks);
			let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor { label: Some("lm-scan-add"), timestamp_writes: None });
			pass.set_pipeline(&self.add);
			pass.set_bind_group(0, &bind, &[]);
			pass.dispatch_workgroups(dx, dy, 1);
		}
		// The deepest level has one block: its sums[0] is the grand total.
		encoder.copy_buffer_to_buffer(&levels.last().expect("at least one level").sums, 0, &staging, 0, 4);
		self.queue.submit(Some(encoder.finish()));
		let total = u32::from_le_bytes(crate::map_read(&self.device, &staging).try_into().expect("4-byte total"));
		let out = levels.swap_remove(0).out;
		(out, total)
	}
}
