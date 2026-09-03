// Copyright (c) LMCAD. Licensed under the MIT License.

//! The GPU field evaluator: compile a [`GpuNode`] tree once, then evaluate the
//! signed distance at arbitrary probe points in bulk.
//!
//! This is the parity-harness workhorse and the building block for any "sample
//! a million points" workload. Timing note for benchmarks: [`GpuField::eval`]
//! is **end-to-end** (point upload, dispatch, readback) — there is no hidden
//! caching, so throughput numbers from it are honest transfer-inclusive rates.

use kernel_core::math::Vec3;
use wgpu::util::DeviceExt;

use crate::codegen::lower;
use crate::tree::GpuNode;
use crate::{GpuContext, GpuError};

/// WGSL entry point appended to the generated field module: one thread per
/// probe point.
const EVAL_ENTRY: &str = r#"
struct LmEvalParams { n: u32, pad0: u32, pad1: u32, pad2: u32 }
@group(1) @binding(0) var<uniform> lm_ep: LmEvalParams;
@group(1) @binding(1) var<storage, read> lm_pts: array<vec4f>;
@group(1) @binding(2) var<storage, read_write> lm_out: array<f32>;

@compute @workgroup_size(256)
fn lm_eval(@builtin(global_invocation_id) gid: vec3u) {
	let i = gid.x;
	if (i >= lm_ep.n) {
		return;
	}
	lm_out[i] = lm_field(lm_pts[i].xyz);
}
"#;

/// Probe points per dispatch: 65535 workgroups × 256 threads (the per-dimension
/// dispatch limit); longer slices are processed in chunks.
const CHUNK: usize = 65_535 * 256;

/// A compiled GPU field: the lowered WGSL pipeline plus its scene buffers.
pub struct GpuField {
	device: wgpu::Device,
	queue: wgpu::Queue,
	pipeline: wgpu::ComputePipeline,
	scene_bind: wgpu::BindGroup,
	io_layout: wgpu::BindGroupLayout,
}

impl GpuField {
	/// Lower `tree` to WGSL and compile the evaluation pipeline.
	pub fn compile(ctx: &GpuContext, tree: &GpuNode) -> Result<GpuField, GpuError> {
		let lowered = lower(tree)?;
		let wgsl = format!("{}{}", lowered.wgsl, EVAL_ENTRY);
		let (scene_layout, scene_bind) = ctx.scene_resources(&lowered);
		let io_layout = ctx.device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			label: Some("lm-eval-io"),
			entries: &[
				GpuContext::uniform_entry(0),
				GpuContext::storage_entry(1, true),
				GpuContext::storage_entry(2, false),
			],
		});
		let pipelines = ctx.compile_pipelines(&wgsl, &[("lm_eval", &[&scene_layout, &io_layout])])?;
		Ok(GpuField {
			device: ctx.device.clone(),
			queue: ctx.queue.clone(),
			pipeline: pipelines.into_iter().next().expect("one pipeline requested"),
			scene_bind,
			io_layout,
		})
	}

	/// Evaluate the field at every probe point (chunked, end-to-end: upload,
	/// dispatch, readback). Result order matches `points`.
	pub fn eval(&self, points: &[Vec3]) -> Vec<f32> {
		let mut out = Vec::with_capacity(points.len());
		for chunk in points.chunks(CHUNK) {
			self.eval_chunk(chunk, &mut out);
		}
		out
	}

	fn eval_chunk(&self, points: &[Vec3], out: &mut Vec<f32>) {
		if points.is_empty() {
			return;
		}
		let n = points.len();
		let packed: Vec<[f32; 4]> = points.iter().map(|p| [p.x, p.y, p.z, 0.0]).collect();
		let pts = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
			label: Some("lm-eval-pts"),
			contents: bytemuck::cast_slice(&packed),
			usage: wgpu::BufferUsages::STORAGE,
		});
		let params = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
			label: Some("lm-eval-params"),
			contents: bytemuck::cast_slice(&[n as u32, 0, 0, 0]),
			usage: wgpu::BufferUsages::UNIFORM,
		});
		let out_buf = self.device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("lm-eval-out"),
			size: (n * 4) as u64,
			usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_SRC,
			mapped_at_creation: false,
		});
		let staging = self.device.create_buffer(&wgpu::BufferDescriptor {
			label: Some("lm-eval-staging"),
			size: (n * 4) as u64,
			usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST,
			mapped_at_creation: false,
		});
		let io_bind = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
			label: Some("lm-eval-io"),
			layout: &self.io_layout,
			entries: &[
				wgpu::BindGroupEntry { binding: 0, resource: params.as_entire_binding() },
				wgpu::BindGroupEntry { binding: 1, resource: pts.as_entire_binding() },
				wgpu::BindGroupEntry { binding: 2, resource: out_buf.as_entire_binding() },
			],
		});

		let mut encoder = self.device.create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("lm-eval") });
		{
			let mut pass = encoder.begin_compute_pass(&wgpu::ComputePassDescriptor { label: Some("lm-eval"), timestamp_writes: None });
			pass.set_pipeline(&self.pipeline);
			pass.set_bind_group(0, &self.scene_bind, &[]);
			pass.set_bind_group(1, &io_bind, &[]);
			pass.dispatch_workgroups(n.div_ceil(256) as u32, 1, 1);
		}
		encoder.copy_buffer_to_buffer(&out_buf, 0, &staging, 0, (n * 4) as u64);
		self.queue.submit(Some(encoder.finish()));
		let bytes = crate::map_read(&self.device, &staging);
		out.extend_from_slice(bytemuck::cast_slice::<u8, f32>(&bytes));
	}
}
