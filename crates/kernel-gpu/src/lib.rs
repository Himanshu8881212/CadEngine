// Copyright (c) LMCAD. Licensed under the MIT License.

//! `kernel-gpu` — wgpu (Metal/Vulkan/DX12) field evaluation and Surface Nets
//! extraction for the implicit half.
//!
//! # Contract (load-bearing, mirrored in `NUMERICS.md`)
//!
//! - **The CPU stays bit-authoritative.** Every [`GpuNode`] tree converts to
//!   the ordinary CPU [`Node`](kernel_implicit::Node) via [`GpuNode::to_node`];
//!   that evaluation (and the CPU meshers consuming it — Manifold DC for
//!   watertight output) is the kernel's source of truth.
//! - **The GPU is tolerance-equivalent, never authoritative**: the lowered
//!   WGSL mirrors each CPU formula branch-for-branch and must satisfy
//!   `|gpu − cpu_f32| ≤ 1e-4 · (1 + |cpu_f32|)` at every probe — enforced by
//!   the parity suite in `tests/parity.rs`.
//! - **GPU extraction is the preview/bulk path.** [`GpuSurfaceNets`] mirrors
//!   `kernel_core::surface_nets` (same lattice layout, same topology rules,
//!   prefix-sum compaction instead of serial emission); its output is closed by
//!   the same shared-corner-sample argument, but the watertight *authority*
//!   remains the CPU's Manifold Dual Contouring. [`narrow_band`] adds the
//!   band-limited variant of the same extraction (work scales with
//!   surface-straddling blocks, not the dense lattice) under the identical
//!   contract.
//! - **No adapter → loud, structured failure** ([`GpuError::NoAdapter`]);
//!   tests runtime-skip with a loud message rather than failing.
//!
//! Everything here evaluates in f32 (WGSL has no f64), including
//! [`Expr`](kernel_implicit::Expr) leaves that the CPU evaluates in f64 — the
//! large-coordinate guidance of `NUMERICS.md`'s f32 section applies to the GPU
//! path wholesale.

mod codegen;
mod eval;
mod extract;
pub mod narrow_band;
mod scan;
mod tree;

pub use eval::GpuField;
pub use extract::{gpu_surface_nets, GpuSurfaceNets};
pub use narrow_band::{extract_narrow_band, extract_narrow_band_with_stats, GpuNarrowBand, NarrowBandStats};
pub use tree::GpuNode;

/// Errors from GPU lowering and execution. Loud by design: no silent CPU
/// fallback happens inside this crate — the caller decides how to degrade.
#[derive(Debug)]
pub enum GpuError {
	/// No usable GPU adapter (e.g. headless CI without Metal/Vulkan).
	NoAdapter(String),
	/// The adapter refused the device request.
	Device(String),
	/// The tree cannot be lowered to WGSL (the message names the leaf and why).
	Lower(String),
	/// The generated shader failed validation — a codegen bug; the message
	/// carries the compiler error plus the full WGSL for diagnosis.
	Shader(String),
	/// The requested extraction exceeds a GPU buffer limit. The CPU dense
	/// meshers silently return an empty mesh over their cap (documented sharp
	/// edge); this NEW api refuses loudly instead.
	TooLarge { what: &'static str, needed_bytes: u64, limit_bytes: u64 },
}

impl std::fmt::Display for GpuError {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		match self {
			GpuError::NoAdapter(e) => write!(f, "no GPU adapter available: {e}"),
			GpuError::Device(e) => write!(f, "GPU device request failed: {e}"),
			GpuError::Lower(e) => write!(f, "cannot lower tree to WGSL: {e}"),
			GpuError::Shader(e) => write!(f, "generated WGSL failed validation (codegen bug): {e}"),
			GpuError::TooLarge { what, needed_bytes, limit_bytes } => {
				write!(f, "extraction too large for this GPU: {what} needs {needed_bytes} bytes, device limit {limit_bytes}")
			}
		}
	}
}

impl std::error::Error for GpuError {}

/// Block until `staging` (a MAP_READ buffer whose copy has been submitted) is
/// mappable, then return its bytes.
pub(crate) fn map_read(device: &wgpu::Device, staging: &wgpu::Buffer) -> Vec<u8> {
	let slice = staging.slice(..);
	let (tx, rx) = std::sync::mpsc::channel();
	slice.map_async(wgpu::MapMode::Read, move |r| {
		let _ = tx.send(r);
	});
	device.poll(wgpu::PollType::wait()).expect("GPU poll failed");
	rx.recv().expect("map_async callback dropped").expect("buffer map failed");
	let data = slice.get_mapped_range().to_vec();
	staging.unmap();
	data
}

/// A wgpu device + queue. One context can serve any number of [`GpuField`]s
/// and [`GpuSurfaceNets`] extractors (wgpu handles are internally
/// reference-counted, so the structs clone what they need).
pub struct GpuContext {
	pub device: wgpu::Device,
	pub queue: wgpu::Queue,
	/// Adapter identity for benchmark/report headers (e.g. "Apple M3 / Metal").
	pub adapter_info: wgpu::AdapterInfo,
}

impl GpuContext {
	/// Acquire the platform's high-performance adapter and a device whose
	/// buffer limits are raised to the adapter's own (extraction buffers can
	/// exceed wgpu's 128 MiB default binding cap on fine grids).
	pub fn new() -> Result<GpuContext, GpuError> {
		let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor::default());
		let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
			power_preference: wgpu::PowerPreference::HighPerformance,
			..Default::default()
		}))
		.map_err(|e| GpuError::NoAdapter(e.to_string()))?;
		let alimits = adapter.limits();
		let limits = wgpu::Limits {
			max_storage_buffer_binding_size: alimits.max_storage_buffer_binding_size,
			max_buffer_size: alimits.max_buffer_size,
			..wgpu::Limits::default()
		};
		let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
			label: Some("lmcad-kernel-gpu"),
			required_features: wgpu::Features::empty(),
			required_limits: limits,
			memory_hints: wgpu::MemoryHints::default(),
			trace: wgpu::Trace::Off,
		}))
		.map_err(|e| GpuError::Device(e.to_string()))?;
		Ok(GpuContext { device, queue, adapter_info: adapter.get_info() })
	}

	/// Create a shader module + one compute pipeline per `(entry_point, bind
	/// group layouts)` pair, inside a validation error scope that converts
	/// validation failures into [`GpuError::Shader`] with the offending WGSL
	/// attached (instead of wgpu's deferred panic). Per-entry layouts let each
	/// pass declare only the bindings it uses (the per-stage storage-buffer
	/// limit counts layout entries, not shader declarations).
	pub(crate) fn compile_pipelines(
		&self,
		wgsl: &str,
		entries: &[(&str, &[&wgpu::BindGroupLayout])],
	) -> Result<Vec<wgpu::ComputePipeline>, GpuError> {
		self.device.push_error_scope(wgpu::ErrorFilter::Validation);
		let module = self.device.create_shader_module(wgpu::ShaderModuleDescriptor {
			label: Some("lmcad-kernel-gpu"),
			source: wgpu::ShaderSource::Wgsl(wgsl.into()),
		});
		let pipelines = entries
			.iter()
			.map(|(entry, layouts)| {
				let layout = self.device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
					label: Some(entry),
					bind_group_layouts: layouts,
					push_constant_ranges: &[],
				});
				self.device.create_compute_pipeline(&wgpu::ComputePipelineDescriptor {
					label: Some(entry),
					layout: Some(&layout),
					module: &module,
					entry_point: Some(entry),
					compilation_options: Default::default(),
					cache: None,
				})
			})
			.collect();
		if let Some(e) = pollster::block_on(self.device.pop_error_scope()) {
			return Err(GpuError::Shader(format!("{e}\n---- generated WGSL ----\n{wgsl}")));
		}
		Ok(pipelines)
	}

	/// A storage-buffer bind group layout entry.
	pub(crate) fn storage_entry(binding: u32, read_only: bool) -> wgpu::BindGroupLayoutEntry {
		wgpu::BindGroupLayoutEntry {
			binding,
			visibility: wgpu::ShaderStages::COMPUTE,
			ty: wgpu::BindingType::Buffer {
				ty: wgpu::BufferBindingType::Storage { read_only },
				has_dynamic_offset: false,
				min_binding_size: None,
			},
			count: None,
		}
	}

	/// A uniform-buffer bind group layout entry.
	pub(crate) fn uniform_entry(binding: u32) -> wgpu::BindGroupLayoutEntry {
		wgpu::BindGroupLayoutEntry {
			binding,
			visibility: wgpu::ShaderStages::COMPUTE,
			ty: wgpu::BindingType::Buffer {
				ty: wgpu::BufferBindingType::Uniform,
				has_dynamic_offset: false,
				min_binding_size: None,
			},
			count: None,
		}
	}

	/// The scene bind group (group 0): strut + grid storage buffers from a
	/// lowered tree, with tiny zeroed dummies when a buffer is unused (the
	/// layout always declares both so every generated module binds uniformly).
	pub(crate) fn scene_resources(&self, lowered: &codegen::Lowered) -> (wgpu::BindGroupLayout, wgpu::BindGroup) {
		use wgpu::util::DeviceExt;
		let layout = self.device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
			label: Some("lm-scene"),
			entries: &[Self::storage_entry(0, true), Self::storage_entry(1, true)],
		});
		let dummy_strut = [[0.0f32; 8]];
		let struts: &[[f32; 8]] = if lowered.struts.is_empty() { &dummy_strut } else { &lowered.struts };
		let dummy_grid = [0.0f32];
		let grid: &[f32] = if lowered.grid_data.is_empty() { &dummy_grid } else { &lowered.grid_data };
		let strut_buf = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
			label: Some("lm-struts"),
			contents: bytemuck::cast_slice(struts),
			usage: wgpu::BufferUsages::STORAGE,
		});
		let grid_buf = self.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
			label: Some("lm-grid"),
			contents: bytemuck::cast_slice(grid),
			usage: wgpu::BufferUsages::STORAGE,
		});
		let bind = self.device.create_bind_group(&wgpu::BindGroupDescriptor {
			label: Some("lm-scene"),
			layout: &layout,
			entries: &[
				wgpu::BindGroupEntry { binding: 0, resource: strut_buf.as_entire_binding() },
				wgpu::BindGroupEntry { binding: 1, resource: grid_buf.as_entire_binding() },
			],
		});
		(layout, bind)
	}
}
