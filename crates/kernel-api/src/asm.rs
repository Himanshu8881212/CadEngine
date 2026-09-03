// Copyright (c) LMCAD. Licensed under the MIT License.

//! The `.lmcasm` executable surface (`kernel-api asm <file.lmcasm>`): load an
//! assembly file, re-solve its mates, export the assembled (and every named
//! state's) merged mesh plus one STL per instance, emit the BOM, and run the
//! contact/clearance scan — all reported as the same machine-readable
//! [`Report`] the `run` subcommand emits (exit 0 iff every step succeeded).
//!
//! This is the official replacement for the retired `tools/asmcheck` workaround
//! harness (FRICTION.md #1): everything that harness had to reach through the
//! Rust API for is now a CLI step with structured output.

use std::collections::BTreeSet;
use std::path::{Path, PathBuf};

use kernel_core::Mesh;
use kernel_model::format::{load_assembly, LoadedAssembly};
use kernel_model::MeshRoute;
use serde_json::{json, Value};

use crate::report::{ErrorKind, OpError, OpReport, Report};

/// Mate residual above which the `mates` step fails: a loaded assembly whose
/// mates did not re-solve is not the assembly the file describes.
const MAX_MATE_RESIDUAL: f64 = 1e-6;

/// Distance at or below which a proximity pair counts as **touching** (the
/// designed-contact / interference class, as opposed to a near fit).
const CONTACT_EPS: f64 = 1e-6;

/// Tuning knobs of [`run_assembly`]; `Default` gives the documented CLI defaults.
#[derive(Clone, Debug)]
pub struct AsmOptions {
	/// Directory `path` part sources resolve against; `None` = the assembly
	/// file's own directory (the `.lmcasm` contract).
	pub base_dir: Option<PathBuf>,
	/// Chord tolerance (mm) for the exact tessellation of B-rep parts.
	pub tol: f64,
	/// Voxel size (mm) for organic/implicit parts and the watertight heal.
	pub voxel: f64,
	/// Proximity window (mm): pairs closer than this are listed with their
	/// measured distance (touching pairs are the subset at distance ≤ 1e-6).
	pub window: f64,
}

impl Default for AsmOptions {
	fn default() -> Self {
		AsmOptions { base_dir: None, tol: 0.05, voxel: 0.4, window: 1.0 }
	}
}

/// Shorthand failing [`OpReport`].
fn fail(id: &str, kind: ErrorKind, message: String) -> OpReport {
	OpReport { id: id.to_string(), ok: false, measures: None, warnings: Vec::new(), file: None, error: Some(OpError { kind, message }) }
}

/// Shorthand passing [`OpReport`].
fn pass(id: &str, measures: Value, file: Option<String>) -> OpReport {
	OpReport { id: id.to_string(), ok: true, measures: Some(measures), warnings: Vec::new(), file, error: None }
}

/// `name` reduced to a filesystem-safe stem (`[A-Za-z0-9_-]`, never empty).
fn sanitize(name: &str) -> String {
	let s: String = name.chars().map(|c| if c.is_ascii_alphanumeric() || c == '-' || c == '_' { c } else { '_' }).collect();
	if s.is_empty() {
		"assembly".to_string()
	} else {
		s
	}
}

/// Join `file` onto `out_dir` and create parent directories.
fn out_path(out_dir: &Path, file: &str) -> std::io::Result<PathBuf> {
	let path = out_dir.join(file);
	if let Some(parent) = path.parent() {
		if !parent.as_os_str().is_empty() {
			std::fs::create_dir_all(parent)?;
		}
	}
	Ok(path)
}

/// The display name of instance `index` (its file name when unnamed).
fn instance_name(loaded: &LoadedAssembly, index: usize) -> String {
	loaded.instance_names.get(index).and_then(Clone::clone).unwrap_or_else(|| format!("#{index}"))
}

/// Append `src` onto `dst`, rebasing indices (merged-export accumulator).
fn append_mesh(dst: &mut Mesh, src: &Mesh) {
	let base = dst.positions.len() as u32;
	dst.positions.extend_from_slice(&src.positions);
	dst.normals.extend_from_slice(&src.normals);
	dst.indices.extend(src.indices.iter().map(|&i| i + base));
}

/// The `run` report vocabulary for a mesh route.
fn route_name(route: MeshRoute) -> &'static str {
	match route {
		MeshRoute::Exact => "exact",
		MeshRoute::Healed => "voxel_healed",
	}
}

/// Execute the full assembly pipeline for `asm_path`, writing exports under
/// `out_dir`, and return the structured [`Report`] (exit-0 contract: `ok` is
/// true iff every step succeeded):
///
/// 1. `load` — parse + instantiate the `.lmcasm` (parts rebuilt, sub-assemblies
///    resolved recursively and flattened to leaf parts, poses applied);
/// 2. `mates` — the on-load mate re-solve residual (the max across all nesting
///    levels), gated at `1e-6`, with HONESTY receipts: statically broken mates
///    refuse (`invalid_param`), `per_mate` residuals name which mate is
///    unsatisfied, and `dof` reports the remaining free rigid-body motions
///    (numeric Jacobian rank — an under-constrained assembly is visibly
///    different from a fully-mated one);
/// 3. `bom` — the BOM v2 payload (`schema: "bom/2"`, grouped `flat` lines with
///    part-number/material/mass enrichment where the part envelopes carry
///    `meta`, plus the nesting `tree` with rolled-up counts), written to
///    `bom.json` and — flat view — `bom.csv`;
/// 4. `export:<NN>:<instance>` — one world-posed STL per unsuppressed leaf
///    instance (exact tessellation when the part's B-rep allows, with the route
///    named); sub-assembly members are named hierarchically
///    (`stage1/bearing_l`);
/// 5. `export:assembly` — the merged assembled mesh, then
///    `export:assembly_step` — the AP214 STEP assembly (NAUO tree, solved
///    poses, volume-conserving) for every B-rep-backed instance, with
///    mesh/organic instances listed as honestly `skipped`;
/// 6. `contacts` — the proximity scan: every pair of leaf instances closer than
///    `window`, with measured distances (`touching` counts the `distance ≤ 1e-6`
///    subset — the designed-contact / interference class);
/// 7. `state:<name>` — every named state applied and exported as a merged STL.
pub fn run_assembly(asm_path: &Path, out_dir: &Path, opts: &AsmOptions) -> Report {
	// V5 panic boundary: the assembly path runs the kernel loader, mates, per-instance
	// mesh/booleans and BOM — any could panic on a hostile or degenerate input. Unlike the
	// op interpreter's `run_one` (interp.rs), this path had NO catch_unwind (audit V5), so a
	// kernel panic here would crash the shared process. Wrap the body so any panic becomes a
	// structured ok:false Report instead. (Release profile is panic="unwind", so this catches.)
	guard_assembly(std::panic::AssertUnwindSafe(|| run_assembly_inner(asm_path, out_dir, opts)))
}

/// Run an assembly-producing closure under the panic guard: a panic becomes a structured
/// ok:false Report (ErrorKind::Internal) instead of crashing the process. Both the
/// `run_assembly` boundary and its unit test go through here, so the test exercises the real
/// mechanism rather than a copy.
fn guard_assembly(f: impl FnOnce() -> Report + std::panic::UnwindSafe) -> Report {
	match std::panic::catch_unwind(f) {
		Ok(report) => report,
		Err(payload) => Report { ok: false, ops: vec![fail("$assembly", ErrorKind::Internal, format!("assembly: kernel panic: {}", panic_detail(payload)))] },
	}
}

/// Extract a human-readable message from a caught panic payload (mirrors interp.rs).
fn panic_detail(payload: Box<dyn std::any::Any + Send>) -> String {
	payload
		.downcast_ref::<&str>()
		.map(|s| (*s).to_string())
		.or_else(|| payload.downcast_ref::<String>().cloned())
		.unwrap_or_else(|| "<non-string panic payload>".to_string())
}

fn run_assembly_inner(asm_path: &Path, out_dir: &Path, opts: &AsmOptions) -> Report {
	let mut ops: Vec<OpReport> = Vec::new();
	let mut all_ok = true;

	// --- 1. load -------------------------------------------------------------
	let text = match std::fs::read_to_string(asm_path) {
		Ok(t) => t,
		Err(e) => {
			return Report {
				ok: false,
				ops: vec![fail("load", ErrorKind::Io, format!("cannot read assembly '{}': {e}", asm_path.display()))],
			};
		}
	};
	let default_base = asm_path.parent().map(Path::to_path_buf).unwrap_or_else(|| PathBuf::from("."));
	let base_dir = opts.base_dir.clone().unwrap_or(default_base);
	let mut loaded = match load_assembly(&text, &base_dir) {
		Ok(l) => l,
		Err(e) => {
			return Report {
				ok: false,
				ops: vec![fail(
					"load",
					ErrorKind::InvalidParam,
					format!("'{}' is not a loadable .lmcasm (parts resolved against '{}'): {e}", asm_path.display(), base_dir.display()),
				)],
			};
		}
	};
	let stem = sanitize(if loaded.name.is_empty() {
		asm_path.file_stem().and_then(|s| s.to_str()).unwrap_or("assembly")
	} else {
		&loaded.name
	});
	let n = loaded.assembly.instances.len();
	let suppressed: Vec<usize> = (0..n).filter(|&i| loaded.assembly.is_instance_suppressed(i)).collect();
	ops.push(pass(
		"load",
		json!({
			"name": loaded.name,
			"units": loaded.units,
			// Leaf parts after flattening sub-assemblies; for a flat assembly
			// this equals top_level.
			"instances": n,
			// The file's own instance count (parts + whole sub-assemblies).
			"top_level": loaded.tree.len(),
			"suppressed": suppressed,
			"mates": loaded.mates.len(),
			"states": loaded.states.keys().cloned().collect::<Vec<_>>(),
		}),
		None,
	));

	// --- 2. mates ------------------------------------------------------------
	// Honesty receipts (assembly audit 2026-07-17): statically broken mates
	// REFUSE (the solver would silently skip them), per-mate residuals name the
	// culprit on failure, and the numeric DOF report makes an under-constrained
	// assembly visibly different from a fully-mated one.
	let (mate_problems, per_mate, dof) = loaded.mate_receipts();
	let per_mate_json: Vec<Value> = per_mate
		.iter()
		.enumerate()
		.map(|(i, &r)| json!({ "index": i, "kind": loaded.mates[i].kind_name(), "residual": r }))
		.collect();
	if !mate_problems.is_empty() {
		all_ok = false;
		ops.push(fail(
			"mates",
			ErrorKind::InvalidParam,
			format!("statically broken mates (refused, not silently skipped): {}", mate_problems.join("; ")),
		));
	} else if loaded.residual <= MAX_MATE_RESIDUAL {
		ops.push(pass(
			"mates",
			json!({
				"residual": loaded.residual,
				"max_residual": MAX_MATE_RESIDUAL,
				"per_mate": per_mate_json,
				"dof": serde_json::to_value(&dof).expect("DofReport serializes: plain data"),
			}),
			None,
		));
	} else {
		all_ok = false;
		let mut worst: Vec<(usize, f64)> = per_mate.iter().copied().enumerate().collect();
		worst.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
		let culprits = worst
			.iter()
			.take(3)
			.filter(|(_, r)| *r > MAX_MATE_RESIDUAL)
			.map(|&(i, r)| format!("mate {i} ({}) residual {r:.3e}", loaded.mates[i].kind_name()))
			.collect::<Vec<_>>()
			.join(", ");
		ops.push(fail(
			"mates",
			ErrorKind::AssertFailed,
			format!(
				"mates did not re-solve: residual {:.3e} exceeds {MAX_MATE_RESIDUAL:.0e} — worst offenders: {culprits}. \
				 The mate set is unsatisfiable (conflicting mates) or stuck in a rotational local optimum; \
				 {} of {} rows are redundant ({})",
				loaded.residual,
				dof.redundant_rows,
				dof.constraint_rows,
				dof.verdict
			),
		));
	}

	// --- 3. BOM (v2: flat + tree → bom.json, flat → bom.csv) -------------------
	let bom = loaded.bom_v2(opts.voxel as f32);
	let bom_value: Value = serde_json::to_value(&bom).expect("a BOM serializes: plain data");
	let json_write = out_path(out_dir, "bom.json").and_then(|p| std::fs::write(&p, bom.to_json()).map(|()| p));
	let csv_write = out_path(out_dir, "bom.csv").and_then(|p| std::fs::write(&p, bom.to_csv()).map(|()| p));
	match (json_write, csv_write) {
		(Ok(json_path), Ok(csv_path)) => {
			let mut measures = bom_value;
			measures["csv"] = json!(csv_path.display().to_string());
			ops.push(pass("bom", measures, Some(json_path.display().to_string())));
		}
		(json_write, csv_write) => {
			all_ok = false;
			let e = json_write.err().or(csv_write.err()).expect("one of the two writes failed");
			ops.push(fail("bom", ErrorKind::Io, format!("cannot write bom.json/bom.csv under '{}': {e}", out_dir.display())));
		}
	}

	// --- 4. per-instance exports (collected for the merged export) -------------
	let mut merged = Mesh::new();
	let mut merged_count = 0usize;
	for i in 0..n {
		let name = instance_name(&loaded, i);
		let id = format!("export:{i:02}:{name}");
		if loaded.assembly.is_instance_suppressed(i) {
			ops.push(pass(&id, json!({ "suppressed": true }), None));
			continue;
		}
		let Some((mesh, route)) = loaded.assembly.mesh_instance_exact_routed(i, opts.tol, opts.voxel as f32) else {
			all_ok = false;
			ops.push(fail(
				&id,
				ErrorKind::InvalidGeometry,
				format!("instance {i} ('{name}') produced no geometry — its document evaluates to nothing"),
			));
			continue;
		};
		let file = format!("parts/{i:02}_{}.stl", sanitize(&name));
		match out_path(out_dir, &file).and_then(|p| mesh.write_stl_binary(&p).map(|()| p)) {
			Ok(path) => {
				append_mesh(&mut merged, &mesh);
				merged_count += 1;
				ops.push(pass(
					&id,
					json!({
						"part": loaded.part_names.get(i).cloned().unwrap_or_default(),
						"triangles": mesh.triangle_count(),
						"watertight": mesh.is_watertight(),
						"route": route_name(route.route),
					}),
					Some(path.display().to_string()),
				));
			}
			Err(e) => {
				all_ok = false;
				ops.push(fail(&id, ErrorKind::Io, format!("cannot write '{file}': {e}")));
			}
		}
	}

	// --- 5. merged assembled export ---------------------------------------------
	if merged.triangle_count() == 0 {
		all_ok = false;
		ops.push(fail(
			"export:assembly",
			ErrorKind::InvalidGeometry,
			"the assembly meshed to nothing (no unsuppressed instance produced geometry)".to_string(),
		));
	} else {
		let file = format!("{stem}_assembly.stl");
		match out_path(out_dir, &file).and_then(|p| merged.write_stl_binary(&p).map(|()| p)) {
			Ok(path) => ops.push(pass(
				"export:assembly",
				json!({
					"instances": merged_count,
					"triangles": merged.triangle_count(),
					"watertight": merged.is_watertight(),
				}),
				Some(path.display().to_string()),
			)),
			Err(e) => {
				all_ok = false;
				ops.push(fail("export:assembly", ErrorKind::Io, format!("cannot write '{file}': {e}")));
			}
		}
	}

	// --- 5b. STEP assembly export (audit gap 10: the volume-conserving AP214
	// NAUO exporter existed in kernel-brep since 2026-07-02 but was unreachable
	// from this pipeline — assemblies exported STL only). B-rep-backed
	// instances export at their SOLVED poses; mesh/organic instances have no
	// B-rep and are listed as honestly skipped (STEP carries no tessellation
	// here). No B-rep instance at all ⇒ the step reports that instead of
	// writing an empty file.
	{
		let mut step_parts: Vec<(String, kernel_brep::Solid, kernel_core::math::DAffine3)> = Vec::new();
		let mut skipped: Vec<Value> = Vec::new();
		for i in 0..n {
			if loaded.assembly.is_instance_suppressed(i) {
				continue;
			}
			let name = instance_name(&loaded, i);
			match &loaded.assembly.instances[i].source {
				kernel_model::Source::Doc(doc) => match doc.evaluate_brep() {
					Some(solid) => {
						// `.lmcasm` poses are rigid BY CONTRACT (scale refused on
						// load); the f32 pose store leaves ~1e-7 scale noise that
						// the STEP exporter's strict rigidity check would refuse,
						// so rebuild from the unit-normalized rotation only.
						let (_, r, t) = loaded.assembly.instances[i].pose.to_scale_rotation_translation();
						let pose = kernel_core::math::DAffine3::from_rotation_translation(r.as_dquat().normalize(), t.as_dvec3());
						step_parts.push((name, solid, pose));
					}
					None => skipped.push(json!({ "instance": name, "why": "document has no exact B-rep (voxel/implicit part)" })),
				},
				kernel_model::Source::Built(_) => {
					skipped.push(json!({ "instance": name, "why": "mesh/prebuilt instance — no B-rep to write into STEP" }));
				}
			}
		}
		if step_parts.is_empty() {
			ops.push(pass(
				"export:assembly_step",
				json!({ "parts": 0, "skipped": skipped, "note": "no B-rep-backed instance — STEP not written (STL exports above are the deliverable)" }),
				None,
			));
		} else {
			match kernel_brep::export_step_assembly(&step_parts, &loaded.name) {
				Ok(step_text) => {
					let file = format!("{stem}_assembly.step");
					match out_path(out_dir, &file).and_then(|p| std::fs::write(&p, &step_text).map(|()| p)) {
						Ok(path) => ops.push(pass(
							"export:assembly_step",
							json!({ "parts": step_parts.len(), "skipped": skipped, "bytes": step_text.len() }),
							Some(path.display().to_string()),
						)),
						Err(e) => {
							all_ok = false;
							ops.push(fail("export:assembly_step", ErrorKind::Io, format!("cannot write '{file}': {e}")));
						}
					}
				}
				Err(e) => {
					all_ok = false;
					ops.push(fail(
						"export:assembly_step",
						ErrorKind::InvalidGeometry,
						format!("STEP assembly export refused: {e}"),
					));
				}
			}
		}
	}

	// --- 6. contact / clearance scan ----------------------------------------------
	let pairs = loaded.assembly.proximity_pairs(opts.window, opts.tol, opts.voxel as f32);
	let touching: BTreeSet<(usize, usize)> = pairs.iter().filter(|(_, _, d)| *d <= CONTACT_EPS).map(|&(i, j, _)| (i, j)).collect();
	let pair_json: Vec<Value> = pairs
		.iter()
		.map(|&(i, j, d)| {
			json!({
				"a": instance_name(&loaded, i),
				"b": instance_name(&loaded, j),
				"i": i,
				"j": j,
				"distance": d,
				"touching": d <= CONTACT_EPS,
			})
		})
		.collect();
	ops.push(pass(
		"contacts",
		json!({
			"window": opts.window,
			"tol": opts.tol,
			"pairs": pair_json,
			"touching": touching.len(),
		}),
		None,
	));

	// --- 7. named states ---------------------------------------------------------
	let baseline = loaded.assembly.capture_state();
	let state_names: Vec<String> = loaded.states.keys().cloned().collect();
	for state_name in state_names {
		let id = format!("state:{state_name}");
		let state = loaded.states[&state_name].clone();
		if !loaded.assembly.apply_state(&state) {
			// load_assembly validates states against the file, so this would be a bug.
			all_ok = false;
			ops.push(fail(&id, ErrorKind::Internal, format!("state '{state_name}' no longer fits the assembly it was loaded with")));
			continue;
		}
		let mesh = loaded.assembly.mesh_all_exact(opts.tol, opts.voxel as f32);
		if mesh.triangle_count() == 0 {
			all_ok = false;
			ops.push(fail(&id, ErrorKind::InvalidGeometry, format!("state '{state_name}' meshed to nothing")));
			continue;
		}
		let file = format!("{stem}_state_{}.stl", sanitize(&state_name));
		match out_path(out_dir, &file).and_then(|p| mesh.write_stl_binary(&p).map(|()| p)) {
			Ok(path) => ops.push(pass(
				&id,
				json!({
					"triangles": mesh.triangle_count(),
					"watertight": mesh.is_watertight(),
					"suppressed": state.suppressed.len(),
				}),
				Some(path.display().to_string()),
			)),
			Err(e) => {
				all_ok = false;
				ops.push(fail(&id, ErrorKind::Io, format!("cannot write '{file}': {e}")));
			}
		}
	}
	if !loaded.assembly.apply_state(&baseline) {
		all_ok = false;
		ops.push(fail("state:$restore", ErrorKind::Internal, "could not restore the assembled state after state exports".to_string()));
	}

	Report { ok: all_ok, ops }
}

#[cfg(test)]
mod tests {
	use super::*;

	// V5: a panic on the assembly path must become a structured ok:false Report (Internal),
	// not a process crash. A real kernel panic can't be reliably forced through a full
	// assembly from a unit test, so this drives the EXACT guard `run_assembly` uses
	// (`guard_assembly`) with a deliberate panic — proving the mechanism, honestly.
	#[test]
	fn assembly_panic_becomes_a_structured_report_not_a_crash() {
		let report = guard_assembly(std::panic::AssertUnwindSafe(|| -> Report { panic!("boom in a mesh step") }));
		let e = report.ops.first().and_then(|o| o.error.as_ref());
		assert!(
			!report.ok
				&& report.ops.len() == 1
				&& e.map(|e| e.kind) == Some(ErrorKind::Internal)
				&& e.map(|e| e.message.contains("boom")).unwrap_or(false),
			"a panic on the assembly path must yield an ok:false Internal report — {report:#?}"
		);
	}
}
