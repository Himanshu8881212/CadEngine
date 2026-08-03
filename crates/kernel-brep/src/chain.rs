// Copyright (c) LMCAD. Licensed under the MIT License.

//! **Boolean chain debugger** — validate every step, name the first bad one.
//!
//! Campaign parts are built as chains of 15–30 booleans. When step 23 turns
//! the solid invalid (or leaves it valid but with a cracked default
//! tessellation), the raw chain reports nothing until a downstream gate — and
//! the author bisects by hand (the RESPOOL campaign wrote a throwaway harness
//! to do exactly that, 2026-07-28; this module is that harness, promoted and
//! made honest). [`ChainLog`] wraps the running solid: every [`apply`]
//! validates the result — and, with [`seal`](ChainLog::seal) on, also
//! tessellates it and checks watertightness — refusing to continue past the
//! first failing step, which it names.
//!
//! ```no_run
//! # use kernel_brep::{ChainLog, cuboid, cylinder, difference};
//! # use kernel_brep::math::DVec3;
//! let plate = cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(30.0, 20.0, 6.0));
//! let bore = cylinder(DVec3::new(15.0, 10.0, -1.0), DVec3::Z, 4.0, 8.0, 48);
//! let mut chain = ChainLog::start("plate", plate).unwrap();
//! chain.apply("bore", |s| difference(s, &bore)).unwrap();
//! let part = chain.finish();
//! ```

use crate::tessellate::tessellate_default;
use crate::topo::Solid;
use crate::validate::{validate, Validity};

/// Per-step record: what the chain looked like after this op.
#[derive(Clone, Debug)]
pub struct ChainStep {
	pub label: String,
	pub validity: Validity,
	/// `None` when the chain runs without [`ChainLog::seal`]; `Some(wt)` when
	/// the default tessellation was checked.
	pub watertight: Option<bool>,
	pub faces: usize,
}

/// The first failing step of a chain, with everything needed to reproduce it.
#[derive(Debug)]
pub struct ChainError {
	/// Label of the step whose RESULT failed.
	pub label: String,
	/// Zero-based index of the failing step (0 = the starting solid).
	pub step: usize,
	pub validity: Validity,
	/// `Some(false)` when the step validated but its default tessellation is
	/// not watertight (the valid-but-leaky class — see DESIGN_GUIDE §7.6).
	pub watertight: Option<bool>,
}

impl std::fmt::Display for ChainError {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		write!(
			f,
			"chain step {} ('{}') failed: valid={} (closed={} manifold={} genus={}){}",
			self.step,
			self.label,
			self.validity.is_valid(),
			self.validity.closed,
			self.validity.manifold,
			self.validity.genus,
			match self.watertight {
				Some(false) => " — B-rep valid but default tessellation NOT watertight (route via precise_mesh, or fix the input per the boolean-hygiene checklist)",
				_ => "",
			}
		)
	}
}

impl std::error::Error for ChainError {}

/// A running boolean chain that refuses to continue past a bad step.
pub struct ChainLog {
	current: Solid,
	steps: Vec<ChainStep>,
	seal: bool,
}

impl ChainLog {
	/// Start a chain from `solid`, validating it as step 0.
	pub fn start(label: &str, solid: Solid) -> Result<Self, ChainError> {
		let mut chain = ChainLog { current: solid, steps: Vec::new(), seal: false };
		chain.record(label)?;
		Ok(chain)
	}

	/// Also tessellate and check watertightness after every step (catches the
	/// valid-but-leaky class at the step that introduces it, for the cost of
	/// one `tessellate_default` per op). The most recent step — the running
	/// solid — is back-filled immediately, so `start(..)?.seal()` seals the
	/// starting solid too.
	pub fn seal(mut self) -> Self {
		self.seal = true;
		if let Some(last) = self.steps.last_mut() {
			last.watertight = Some(tessellate_default(&self.current).is_watertight());
		}
		self
	}

	fn record(&mut self, label: &str) -> Result<(), ChainError> {
		let validity = validate(&self.current);
		let watertight = if self.seal { Some(tessellate_default(&self.current).is_watertight()) } else { None };
		let ok = validity.is_valid() && watertight != Some(false);
		self.steps.push(ChainStep {
			label: label.to_string(),
			validity,
			watertight,
			faces: self.current.face_count(),
		});
		if ok {
			Ok(())
		} else {
			Err(ChainError { label: label.to_string(), step: self.steps.len() - 1, validity, watertight })
		}
	}

	/// Apply one op to the running solid and validate the result. On failure
	/// the chain keeps the PRE-step solid (inspect it via [`solid`], re-try a
	/// repaired cutter, or dump operands for a repro).
	pub fn apply(&mut self, label: &str, op: impl FnOnce(&Solid) -> Solid) -> Result<&Solid, ChainError> {
		// LMCAD_CHAIN_TRACE=1 streams step labels to stderr as they START —
		// when an op spins inside the arrangement (observed: a
		// resolve_t_junctions cascade, DRYBOX 2026-07-28), the trace names the
		// guilty step live instead of leaving a silent 100%-CPU process.
		let trace = std::env::var_os("LMCAD_CHAIN_TRACE").is_some();
		if trace {
			eprintln!("[chain] {} …", label);
		}
		let t0 = std::time::Instant::now();
		let next = op(&self.current);
		let secs = t0.elapsed().as_secs_f32();
		if trace {
			eprintln!("[chain] {} done in {:.1}s ({} faces)", label, secs, next.face_count());
		}
		let prev = std::mem::replace(&mut self.current, next);
		let faces = self.current.face_count();
		let outcome = self.record(label);
		// Level-1 telemetry (opt-in): one "chain_op" event per step outcome —
		// part of the flywheel dataset (`kernel_core::telemetry`). Labels here
		// are simple ASCII; '"' is swapped for '\'' so a stray quote cannot
		// break the JSON line.
		if kernel_core::telemetry::enabled() {
			let (valid, wt) = match &outcome {
				Ok(()) => {
					let s = self.steps.last().expect("record() pushed this step");
					(s.validity.is_valid(), s.watertight)
				}
				Err(e) => (e.validity.is_valid(), e.watertight),
			};
			kernel_core::telemetry::log(
				"chain_op",
				&format!(
					"\"label\":\"{}\",\"secs\":{secs},\"faces\":{faces},\"valid\":{valid},\"watertight\":{}",
					label.replace('"', "'"),
					json_opt_bool(wt),
				),
			);
		}
		match outcome {
			Ok(()) => Ok(&self.current),
			Err(e) => {
				self.current = prev; // keep the last-good solid
				self.steps.pop();
				// A refusal is ALWAYS captured (not gated by `enabled`): the
				// friction inbox is the raw feed the lessons workflow curates.
				kernel_core::telemetry::log_friction(
					"chain_refusal",
					&format!(
						"\"label\":\"{}\",\"valid\":{},\"closed\":{},\"manifold\":{},\"genus\":{},\"watertight\":{}",
						e.label.replace('"', "'"),
						e.validity.is_valid(),
						e.validity.closed,
						e.validity.manifold,
						e.validity.genus,
						json_opt_bool(e.watertight),
					),
				);
				Err(e)
			}
		}
	}

	/// The last-good solid.
	pub fn solid(&self) -> &Solid {
		&self.current
	}

	/// Every recorded (successful) step, in order.
	pub fn steps(&self) -> &[ChainStep] {
		&self.steps
	}

	/// Consume the chain, returning the final solid.
	pub fn finish(self) -> Solid {
		self.current
	}
}

/// The JSON spelling of a tri-state watertightness: `true` / `false` /
/// `null` (`None` = the chain ran without [`ChainLog::seal`]).
fn json_opt_bool(b: Option<bool>) -> &'static str {
	match b {
		Some(true) => "true",
		Some(false) => "false",
		None => "null",
	}
}
