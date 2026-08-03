// Copyright (c) LMCAD. Licensed under the MIT License.

//! The `implicit` op's expression grammar (BAR.md I6): the CSG [`Node`] algebra
//! as a NESTABLE JSON tree, plus the scalar-field math AST, parsed recursively
//! with structured errors that carry the JSON path to the bad subtree.
//!
//! A tree node is either a **leaf** `{"shape": "...", ...}` (sphere / box /
//! cylinder / cone / capsule / torus / plane / gyroid / tpms / beam_lattice /
//! voronoi_lattice / strut_lattice / pipe / pipe_path / helix_pipe / text /
//! expr_sdf) or a **combinator** `{"op": "...", ...}` over child
//! trees (`a` / `b` / `in`). Scalar expressions (the `expr_sdf` leaf's `expr`
//! and the `field` of `offset_by` / `lerp`) are numbers, the variables
//! `"x"`/`"y"`/`"z"`, or `{"op": ...}` math nodes; the `field` alternatively
//! takes a `{"grid": …}` NPY source (simulation data as a grade law) — see
//! `API.md` for the full grammar and the Lipschitz contract.
//!
//! Every parsed scalar expression is collected as a [`FieldProbe`] so the
//! interpreter can sample it over the resolved meshing domain BEFORE meshing:
//! a NaN/∞ value at a probe point is a loud `invalid_param` naming the
//! subtree, never a poisoned voxel grid.

use std::path::Path;
use std::sync::Arc;

use kernel_core::math::{Aabb, Affine3A, DVec3, Vec3};
use kernel_implicit::grid_field::GridField;
use kernel_implicit::strut::{pipe_path, StrutKind, StrutLattice};
use kernel_implicit::text::text_field;
use kernel_implicit::texture::{displaced, Texture};
use kernel_implicit::{
	chamfer_difference, chamfer_union, fillet_difference, fillet_union, scalar_field, BeamLattice, Capsule, Cone,
	Cuboid, Cylinder, Expr, ExprSdf, Gyroid, LatticeCell, Node, Pipe, Plane, ScalarField, Sphere, Torus, Tpms, TpmsKind,
	VoronoiLattice,
};
use serde_json::{Map, Value};

use crate::report::{ErrorKind, OpError};

/// Cap on pattern copies — domain repetition costs one child evaluation per
/// copy per query, so an absurd count is a compute foot-gun, not a feature.
const MAX_PATTERN_COUNT: usize = 4096;

/// Cap on `beam_lattice` cells (`from_cells` form) — each cubic/octet cell
/// contributes 12/36 struts, so this bounds construction memory.
const MAX_LATTICE_CELLS: usize = 16384;

/// Cap on pipe/helix polyline segments.
const MAX_PIPE_SEGMENTS: usize = 100_000;

/// Cap on `voronoi_lattice` seed points — the in-kernel Delaunay is O(seeds²),
/// so this bounds construction time (and the resulting foam's mesh size).
const MAX_VORONOI_SEEDS: usize = 4096;

/// Cap on `text` characters — every glyph is tens of capsule segments scanned
/// per query (no acceleration grid), so a book-length string is a compute
/// foot-gun; part labels are short.
const MAX_TEXT_CHARS: usize = 256;

/// The supported leaf shapes, for the unknown-shape error message.
const SHAPES: &str = "sphere, box, cylinder, cone, capsule, torus, plane, gyroid, tpms, beam_lattice, voronoi_lattice, \
strut_lattice, pipe, pipe_path, helix_pipe, text, expr_sdf";

/// The supported combinators, for the unknown-op error message.
const COMBINATORS: &str = "union, intersection, difference, smooth_union, smooth_intersection, smooth_difference, \
fillet_union, fillet_difference, chamfer_union, chamfer_difference, displace, offset, shell, translate, rotate, scale, mirror, \
linear_pattern, circular_pattern, offset_by, lerp";

/// The supported scalar-expression ops, for the unknown-scalar-op message.
const SCALAR_OPS: &str = "add, sub, mul, div, min, max, mod, atan2, neg, abs, sqrt, sin, cos, clamp, length2, length3";

/// A scalar expression that must be probed for finiteness over the meshing
/// domain before extraction (an `expr_sdf` leaf's `expr` or a modulation
/// `field`), tagged with its JSON path for the structured error.
pub struct FieldProbe {
	/// JSON path of the expression subtree (e.g. `expr.a.field`).
	pub path: String,
	/// The compiled expression (shared with the built [`Node`]).
	pub expr: Arc<Expr>,
	/// For an `expr_sdf` leaf: the caller's declared Lipschitz bound `L` (the
	/// field is `expr/L`). `Some` only for `expr_sdf`; `None` for the scalar
	/// fields of `offset_by`/`lerp` (which drive no narrow-band pruning). Used by
	/// [`probe_lipschitz`] to catch an under-declared bound before it silently
	/// tears holes in a narrow-band mesh.
	pub lipschitz: Option<f64>,
}

/// A fully parsed `implicit` expression tree: the executable [`Node`] plus
/// every scalar expression awaiting its domain probe.
pub struct ParsedTree {
	pub node: Node,
	pub fields: Vec<FieldProbe>,
}

/// Parse the `expr` parameter of an `implicit` op into a CSG [`Node`].
/// Errors are `invalid_param` and name `op_id` plus the JSON path of the bad
/// subtree. `input_base` is where a `{"grid": {"path": …}}` scalar-field
/// source resolves its `.npy` file (the same confined input root as
/// `load_part` / `mesh_carve` — absolute paths and `..` are refused).
pub fn parse_tree(op_id: &str, value: &Value, input_base: &Path) -> Result<ParsedTree, OpError> {
	let mut ctx = Ctx { op_id, input_base, fields: Vec::new() };
	let node = ctx.node(value, "expr")?;
	Ok(ParsedTree { node, fields: ctx.fields })
}

/// Sample every collected scalar expression on a 5×5×5 lattice over `domain`
/// (corners, face centers and center included): the first non-finite value is
/// a loud `invalid_param` naming the expression's JSON path and the point.
/// A heuristic guard, not a proof — IEEE poles between probe points are the
/// caller's contract (clamp denominators, keep `sqrt` arguments non-negative).
pub fn probe_fields(op_id: &str, fields: &[FieldProbe], domain: Aabb) -> Result<(), OpError> {
	let (lo, hi) = (domain.min.as_dvec3(), domain.max.as_dvec3());
	for probe in fields {
		for i in 0..5 {
			for j in 0..5 {
				for k in 0..5 {
					let t = DVec3::new(i as f64 / 4.0, j as f64 / 4.0, k as f64 / 4.0);
					let p = lo + (hi - lo) * t;
					let v = probe.expr.eval(p);
					if !v.is_finite() {
						return Err(OpError {
							kind: ErrorKind::InvalidParam,
							message: format!(
								"op '{op_id}': at {}: expression evaluates to {v} at probe point [{}, {}, {}] — the field must stay finite over the meshing domain (clamp denominators, keep sqrt arguments non-negative, never mod by 0)",
								probe.path, p.x, p.y, p.z
							),
						});
					}
				}
			}
		}
	}
	Ok(())
}

/// Multiple of the declared bound the sampled `|∇expr|` may reach before the
/// declaration is judged under-stated. The slack absorbs forward-difference
/// overshoot and the chance a coarse lattice lands just off the true peak; it is
/// wide enough that only a genuine under-declaration (the gradient materially
/// exceeds the bound) trips it, not a truthful bound sampled imperfectly.
const LIPSCHITZ_SLACK: f64 = 1.10;

/// Verify every `expr_sdf` leaf's declared `lipschitz_bound` against the field it
/// wraps, by sampling `|∇expr|` on an 12³ lattice over `domain` (forward
/// differences at `h = diagonal/4096`). An under-declared bound makes the field
/// OVERSTATE distances, so the narrow-band meshers prune blocks that still
/// contain surface — tearing silent holes; this turns that silent failure into a
/// loud, actionable refusal naming the observed lower bound.
///
/// Sampling caveat (honest): this is a heuristic lower bound on the true
/// `sup|∇expr|`, not a proof — a peak between lattice points can be missed. It
/// reliably catches gross under-declarations (declaring `1` for a slope-5 field);
/// it is not a certificate. It runs ONLY for the narrow-band mesher — the dense
/// meshers (`manifold`, `surface_nets`) sample every cell and need no bound.
pub fn probe_lipschitz(op_id: &str, fields: &[FieldProbe], domain: Aabb) -> Result<(), OpError> {
	let (lo, hi) = (domain.min.as_dvec3(), domain.max.as_dvec3());
	let h = ((hi - lo).length() / 4096.0).max(1e-9);
	for probe in fields {
		let Some(bound) = probe.lipschitz else { continue };
		let mut observed = 0.0_f64;
		for i in 0..12 {
			for j in 0..12 {
				for k in 0..12 {
					let t = DVec3::new(i as f64 / 11.0, j as f64 / 11.0, k as f64 / 11.0);
					let p = lo + (hi - lo) * t;
					let v = probe.expr.eval(p);
					let gx = (probe.expr.eval(p + DVec3::new(h, 0.0, 0.0)) - v) / h;
					let gy = (probe.expr.eval(p + DVec3::new(0.0, h, 0.0)) - v) / h;
					let gz = (probe.expr.eval(p + DVec3::new(0.0, 0.0, h)) - v) / h;
					let g = (gx * gx + gy * gy + gz * gz).sqrt();
					if g.is_finite() {
						observed = observed.max(g);
					}
				}
			}
		}
		if observed > bound * LIPSCHITZ_SLACK {
			return Err(OpError {
				kind: ErrorKind::InvalidParam,
				message: format!(
					"op '{op_id}': at {}: declared lipschitz_bound {bound} is UNDER-stated — sampled |∇expr| reaches ≈{observed:.3} over the meshing domain, so the narrow-band mesher would prune surface-bearing blocks and tear holes. Raise lipschitz_bound to ≥ {observed:.3}, or switch to \"mesher\": \"manifold\" (dense, needs no bound).",
					probe.path
				),
			});
		}
	}
	Ok(())
}

/// Recursive-descent parser state: the op id for error messages, the confined
/// input root for `{"grid": …}` NPY sources, plus the collected field probes.
struct Ctx<'a> {
	op_id: &'a str,
	input_base: &'a Path,
	fields: Vec<FieldProbe>,
}

impl Ctx<'_> {
	/// Structured `invalid_param` at a JSON path.
	fn bad(&self, path: &str, msg: impl std::fmt::Display) -> OpError {
		OpError { kind: ErrorKind::InvalidParam, message: format!("op '{}': at {path}: {msg}", self.op_id) }
	}

	// --- JSON field accessors (all errors carry the path) ----------------------

	fn require<'v>(&self, obj: &'v Map<String, Value>, key: &str, path: &str) -> Result<&'v Value, OpError> {
		obj.get(key).ok_or_else(|| self.bad(path, format!("missing required field '{key}'")))
	}

	fn num(&self, obj: &Map<String, Value>, key: &str, path: &str) -> Result<f64, OpError> {
		let n = self
			.require(obj, key, path)?
			.as_f64()
			.ok_or_else(|| self.bad(path, format!("field '{key}' must be a number")))?;
		if !n.is_finite() {
			return Err(self.bad(path, format!("field '{key}' must be finite, got {n}")));
		}
		Ok(n)
	}

	fn positive(&self, obj: &Map<String, Value>, key: &str, path: &str) -> Result<f64, OpError> {
		let n = self.num(obj, key, path)?;
		if n <= 0.0 {
			return Err(self.bad(path, format!("field '{key}' must be > 0, got {n}")));
		}
		Ok(n)
	}

	fn count(&self, obj: &Map<String, Value>, key: &str, path: &str, max: usize) -> Result<usize, OpError> {
		let n = self
			.require(obj, key, path)?
			.as_u64()
			.ok_or_else(|| self.bad(path, format!("field '{key}' must be a non-negative integer")))?;
		if n == 0 || n as usize > max {
			return Err(self.bad(path, format!("field '{key}' must be in 1..={max}, got {n}")));
		}
		Ok(n as usize)
	}

	fn vec3(&self, obj: &Map<String, Value>, key: &str, path: &str) -> Result<Vec3, OpError> {
		let arr = self
			.require(obj, key, path)?
			.as_array()
			.ok_or_else(|| self.bad(path, format!("field '{key}' must be an [x, y, z] array")))?;
		self.vec3_value(arr, key, path)
	}

	fn vec3_value(&self, arr: &[Value], what: &str, path: &str) -> Result<Vec3, OpError> {
		if arr.len() != 3 {
			return Err(self.bad(path, format!("'{what}' must have exactly 3 numbers, got {}", arr.len())));
		}
		let mut out = [0.0f32; 3];
		for (i, v) in arr.iter().enumerate() {
			let n = v.as_f64().ok_or_else(|| self.bad(path, format!("'{what}'[{i}] must be a number")))?;
			if !n.is_finite() {
				return Err(self.bad(path, format!("'{what}'[{i}] must be finite, got {n}")));
			}
			out[i] = n as f32;
		}
		Ok(Vec3::new(out[0], out[1], out[2]))
	}

	/// A non-zero direction, normalized.
	fn axis(&self, obj: &Map<String, Value>, key: &str, path: &str) -> Result<Vec3, OpError> {
		self.vec3(obj, key, path)?
			.try_normalize()
			.ok_or_else(|| self.bad(path, format!("field '{key}' must be a non-zero direction vector")))
	}

	/// A `min`/`max` corner pair forming a valid box.
	fn corner_box(&self, obj: &Map<String, Value>, path: &str) -> Result<(Vec3, Vec3), OpError> {
		let (min, max) = (self.vec3(obj, "min", path)?, self.vec3(obj, "max", path)?);
		if !(min.x < max.x && min.y < max.y && min.z < max.z) {
			return Err(self.bad(path, format!("'min' {min:?} must be strictly below 'max' {max:?} on every axis")));
		}
		Ok((min, max))
	}

	// --- The Node tree ----------------------------------------------------------

	fn node(&mut self, v: &Value, path: &str) -> Result<Node, OpError> {
		let obj = v
			.as_object()
			.ok_or_else(|| self.bad(path, "a tree node must be a JSON object with a 'shape' (leaf) or an 'op' (combinator) field"))?;
		match (obj.get("shape"), obj.get("op")) {
			(Some(shape), None) => {
				let shape = shape.as_str().ok_or_else(|| self.bad(path, "'shape' must be a string"))?;
				self.leaf(obj, shape, path)
			}
			(None, Some(op)) => {
				let op = op.as_str().ok_or_else(|| self.bad(path, "'op' must be a string"))?;
				self.combinator(obj, op, path)
			}
			(Some(_), Some(_)) => Err(self.bad(path, "a tree node takes either 'shape' or 'op', not both")),
			(None, None) => Err(self.bad(path, "a tree node needs a 'shape' (leaf primitive) or an 'op' (combinator) field")),
		}
	}

	fn leaf(&mut self, obj: &Map<String, Value>, shape: &str, path: &str) -> Result<Node, OpError> {
		match shape {
			"sphere" => {
				let (center, radius) = (self.vec3(obj, "center", path)?, self.positive(obj, "radius", path)?);
				Ok(Node::primitive(Sphere::new(center, radius as f32)))
			}
			"box" => {
				let (min, max) = self.corner_box(obj, path)?;
				Ok(Node::primitive(Cuboid::from_corners(min, max)))
			}
			"cylinder" => {
				let (a, b) = (self.vec3(obj, "a", path)?, self.vec3(obj, "b", path)?);
				let radius = self.positive(obj, "radius", path)?;
				Ok(Node::primitive(Cylinder::new(a, b, radius as f32)))
			}
			"cone" => {
				let (a, b) = (self.vec3(obj, "a", path)?, self.vec3(obj, "b", path)?);
				let (ra, rb) = (self.num(obj, "ra", path)?, self.num(obj, "rb", path)?);
				if ra < 0.0 || rb < 0.0 || ra + rb <= 0.0 {
					return Err(self.bad(path, format!("cone radii must be >= 0 with at least one > 0, got ra={ra} rb={rb}")));
				}
				Ok(Node::primitive(Cone::new(a, b, ra as f32, rb as f32)))
			}
			"capsule" => {
				let (a, b) = (self.vec3(obj, "a", path)?, self.vec3(obj, "b", path)?);
				let radius = self.positive(obj, "radius", path)?;
				Ok(Node::primitive(Capsule::new(a, b, radius as f32)))
			}
			"torus" => {
				let (center, axis) = (self.vec3(obj, "center", path)?, self.axis(obj, "axis", path)?);
				let (major, minor) = (self.positive(obj, "major", path)?, self.positive(obj, "minor", path)?);
				Ok(Node::primitive(Torus::new(center, axis, major as f32, minor as f32)))
			}
			"plane" => {
				let (point, normal) = (self.vec3(obj, "point", path)?, self.axis(obj, "normal", path)?);
				Ok(Node::primitive(Plane::new(point, normal)))
			}
			"gyroid" => {
				let (min, max) = self.corner_box(obj, path)?;
				let (scale, thickness) = (self.positive(obj, "scale", path)?, self.positive(obj, "thickness", path)?);
				Ok(Node::primitive(Gyroid::new(Aabb::new(min, max), scale as f32, thickness as f32)))
			}
			"tpms" => {
				// A bounded TPMS lattice in any of the six families, network or sheet
				// mode. Its normalized trig field is a distance BOUND, so it is wrapped
				// with `primitive_bound` — a downstream offset/shell is then honestly
				// flagged approximate (the FieldQuality contract).
				let (min, max) = self.corner_box(obj, path)?;
				let cell = self.positive(obj, "cell", path)?;
				let kind = match self.require(obj, "kind", path)?.as_str() {
					Some("gyroid") => TpmsKind::Gyroid,
					Some("schwarz_p") => TpmsKind::SchwarzP,
					Some("diamond") => TpmsKind::Diamond,
					Some("neovius") => TpmsKind::Neovius,
					Some("schoen_iwp") => TpmsKind::SchoenIwp,
					Some("fischer_koch_s") => TpmsKind::FischerKochS,
					other => {
						return Err(self.bad(
							path,
							format!("'kind' must be one of gyroid|schwarz_p|diamond|neovius|schoen_iwp|fischer_koch_s, got {other:?}"),
						))
					}
				};
				let region = Aabb::new(min, max);
				// network: `level` is the iso-level (0 ≈ 50% solid, default; negative
				// thins the labyrinth). sheet: `level` is the wall half-thickness (> 0).
				match obj.get("mode").and_then(Value::as_str) {
					Some("sheet") => {
						let t = self.positive(obj, "level", path)?;
						Ok(Node::primitive_bound(Tpms::sheet(region, kind, cell as f32, t as f32)))
					}
					Some("network") | None => {
						let level = match obj.get("level") {
							None => 0.0,
							Some(_) => self.num(obj, "level", path)?,
						};
						Ok(Node::primitive_bound(Tpms::network(region, kind, cell as f32, level as f32)))
					}
					Some(other) => Err(self.bad(path, format!("'mode' must be 'network' or 'sheet', got {other:?}"))),
				}
			}
			"beam_lattice" => self.beam_lattice(obj, path),
			"voronoi_lattice" => self.voronoi_lattice(obj, path),
			"strut_lattice" => {
				// A triply-periodic strut lattice (BCC / FCC / octet). The FIELD is
				// periodic over all of space; `min`/`max` is only the meshing-bounds
				// hint (exactly like the `tpms` leaf) — intersect with a box/shroud to
				// close the block, or the meshing domain will cut the struts open.
				// Wrapped `primitive_bound` per the kernel contract: the min-union
				// field understates depth inside strut overlaps, so a downstream
				// offset/shell is honestly flagged approximate.
				let (min, max) = self.corner_box(obj, path)?;
				let (cell, radius) = (self.positive(obj, "cell", path)?, self.positive(obj, "radius", path)?);
				let kind = match self.require(obj, "kind", path)?.as_str() {
					Some("bcc") => StrutKind::Bcc,
					Some("fcc") => StrutKind::Fcc,
					Some("octet") => StrutKind::Octet,
					other => return Err(self.bad(path, format!("'kind' must be one of bcc|fcc|octet, got {other:?}"))),
				};
				Ok(Node::primitive_bound(StrutLattice::new(Aabb::new(min, max), kind, cell as f32, radius as f32)))
			}
			"pipe" => self.pipe(obj, path),
			"pipe_path" => {
				// Uniform-radius capsule chain along a polyline — the strut
				// vocabulary's skeleton/pipe convenience (`kernel_implicit::strut::
				// pipe_path`). The general `pipe` leaf (per-point `radii`, `path` key)
				// remains the tapering form.
				let pts_arr = self
					.require(obj, "points", path)?
					.as_array()
					.ok_or_else(|| self.bad(path, "'points' must be an array of [x, y, z] points"))?;
				if pts_arr.len() < 2 {
					return Err(self.bad(path, format!("'points' needs at least 2 points, got {}", pts_arr.len())));
				}
				if pts_arr.len() - 1 > MAX_PIPE_SEGMENTS {
					return Err(self.bad(path, format!("'points' has {} segments, the cap is {MAX_PIPE_SEGMENTS}", pts_arr.len() - 1)));
				}
				let mut pts = Vec::with_capacity(pts_arr.len());
				for (i, v) in pts_arr.iter().enumerate() {
					let arr = v.as_array().ok_or_else(|| self.bad(path, format!("'points'[{i}] must be an [x, y, z] array")))?;
					pts.push(self.vec3_value(arr, &format!("points[{i}]"), path)?);
				}
				let radius = self.positive(obj, "radius", path)?;
				Ok(Node::primitive(pipe_path(&pts, radius as f32)))
			}
			"text" => {
				// Single-stroke Hershey Simplex text as capsule strokes in the z = 0
				// plane (baseline y = 0, glyphs advance +X, capitals scaled to
				// `height`). Charset is PRE-validated here so an unsupported character
				// is a structured op error naming the character — the kernel's own
				// loud panic is never reachable from JSON.
				let text = self
					.require(obj, "text", path)?
					.as_str()
					.ok_or_else(|| self.bad(path, "'text' must be a string"))?;
				if text.chars().count() > MAX_TEXT_CHARS {
					return Err(self.bad(path, format!("'text' has {} characters, the cap is {MAX_TEXT_CHARS}", text.chars().count())));
				}
				if let Some(c) = text.chars().find(|c| !(c.is_ascii_alphanumeric() || *c == ' ' || *c == '-' || *c == '.')) {
					return Err(self.bad(
						path,
						format!("unsupported character {c:?} in 'text' — the embedded Hershey Simplex set covers A-Z (lowercase folds to uppercase), 0-9, space, '-' and '.'"),
					));
				}
				if !text.chars().any(|c| c != ' ') {
					return Err(self.bad(path, "'text' needs at least one non-space glyph"));
				}
				let (height, stroke) = (self.positive(obj, "height", path)?, self.positive(obj, "stroke_radius", path)?);
				Ok(text_field(text, height as f32, stroke as f32))
			}
			"helix_pipe" => {
				let (center, axis) = (self.vec3(obj, "center", path)?, self.axis(obj, "axis", path)?);
				let (r_helix, radius) = (self.positive(obj, "r_helix", path)?, self.positive(obj, "radius", path)?);
				let (pitch, turns) = (self.num(obj, "pitch", path)?, self.positive(obj, "turns", path)?);
				let samples = match obj.get("samples_per_turn") {
					None => 64,
					Some(_) => self.count(obj, "samples_per_turn", path, 1024)?,
				};
				if samples < 8 {
					return Err(self.bad(path, format!("'samples_per_turn' must be >= 8, got {samples}")));
				}
				let segments = (turns * samples as f64).ceil() as usize;
				if segments > MAX_PIPE_SEGMENTS {
					return Err(self.bad(path, format!("helix needs {segments} segments (turns × samples_per_turn), the cap is {MAX_PIPE_SEGMENTS}")));
				}
				Ok(Node::primitive(Pipe::helix(center, axis, r_helix as f32, pitch as f32, turns as f32, samples, radius as f32)))
			}
			"expr_sdf" => {
				let ev = self.require(obj, "expr", path)?;
				let expr = self.field(ev, &format!("{path}.expr"))?;
				let lipschitz = self.num(obj, "lipschitz_bound", path)?;
				if lipschitz <= 0.0 {
					return Err(self.bad(
						path,
						format!("'lipschitz_bound' must be > 0 (a truthful bound on |∇expr|; the kernel divides the field by it — zero set preserved, slope normalized), got {lipschitz}"),
					));
				}
				// Tag the probe `field()` just registered with the declared bound so
				// `probe_lipschitz` can sample-verify it before a narrow-band mesh.
				if let Some(p) = self.fields.last_mut() {
					p.lipschitz = Some(lipschitz);
				}
				let bounds = match (obj.get("min"), obj.get("max")) {
					(None, None) => None,
					(Some(_), Some(_)) => {
						let (min, max) = self.corner_box(obj, path)?;
						Some(Aabb::new(min, max))
					}
					_ => return Err(self.bad(path, "'min' and 'max' bounds must be given together (or both omitted for an unbounded field)")),
				};
				Ok(Node::primitive(ExprSdf::new(expr, lipschitz, bounds)))
			}
			other => Err(self.bad(path, format!("unknown shape '{other}' — supported shapes: {SHAPES}"))),
		}
	}

	fn beam_lattice(&mut self, obj: &Map<String, Value>, path: &str) -> Result<Node, OpError> {
		if obj.contains_key("nodes") || obj.contains_key("struts") {
			// Explicit graph form.
			let nodes_arr = self
				.require(obj, "nodes", path)?
				.as_array()
				.ok_or_else(|| self.bad(path, "'nodes' must be an array of [x, y, z] points"))?;
			let mut nodes = Vec::with_capacity(nodes_arr.len());
			for (i, v) in nodes_arr.iter().enumerate() {
				let arr = v.as_array().ok_or_else(|| self.bad(path, format!("'nodes'[{i}] must be an [x, y, z] array")))?;
				nodes.push(self.vec3_value(arr, &format!("nodes[{i}]"), path)?);
			}
			let struts_arr = self
				.require(obj, "struts", path)?
				.as_array()
				.ok_or_else(|| self.bad(path, "'struts' must be an array of [node_a, node_b, radius_a, radius_b]"))?;
			if struts_arr.is_empty() {
				return Err(self.bad(path, "'struts' must contain at least one strut"));
			}
			let mut struts = Vec::with_capacity(struts_arr.len());
			for (i, v) in struts_arr.iter().enumerate() {
				let arr = v.as_array().filter(|a| a.len() == 4).ok_or_else(|| {
					self.bad(path, format!("'struts'[{i}] must be a 4-element [node_a, node_b, radius_a, radius_b] array"))
				})?;
				let ia = arr[0].as_u64().ok_or_else(|| self.bad(path, format!("'struts'[{i}][0] must be a node index")))?;
				let ib = arr[1].as_u64().ok_or_else(|| self.bad(path, format!("'struts'[{i}][1] must be a node index")))?;
				if ia as usize >= nodes.len() || ib as usize >= nodes.len() {
					return Err(self.bad(path, format!("'struts'[{i}] references node {} but there are only {} nodes", ia.max(ib), nodes.len())));
				}
				let ra = arr[2].as_f64().unwrap_or(f64::NAN);
				let rb = arr[3].as_f64().unwrap_or(f64::NAN);
				if !(ra.is_finite() && rb.is_finite() && ra > 0.0 && rb > 0.0) {
					return Err(self.bad(path, format!("'struts'[{i}] radii must be positive numbers, got [{}, {}]", arr[2], arr[3])));
				}
				struts.push((ia as u32, ib as u32, ra as f32, rb as f32));
			}
			Ok(Node::primitive(BeamLattice::new(nodes, struts)))
		} else {
			// Cell-fill form.
			let (min, max) = self.corner_box(obj, path)?;
			let cell = match self.require(obj, "cell", path)?.as_str() {
				Some("cubic") => LatticeCell::Cubic,
				Some("octet") => LatticeCell::Octet,
				other => return Err(self.bad(path, format!("'cell' must be \"cubic\" or \"octet\", got {other:?}"))),
			};
			let (cell_size, radius) = (self.positive(obj, "cell_size", path)?, self.positive(obj, "radius", path)?);
			let size = max - min;
			let cells = [size.x, size.y, size.z]
				.iter()
				.map(|s| ((*s as f64 / cell_size).floor() as usize).max(1))
				.product::<usize>();
			if cells > MAX_LATTICE_CELLS {
				return Err(self.bad(path, format!("the region holds {cells} cells of size {cell_size}, the cap is {MAX_LATTICE_CELLS} — coarsen 'cell_size'")));
			}
			Ok(Node::primitive(BeamLattice::from_cells(Aabb::new(min, max), cell, cell_size as f32, radius as f32)))
		}
	}

	/// `{"shape": "voronoi_lattice", "seeds": [[x,y,z],...], "radius": r,
	/// "min": [x,y,z], "max": [x,y,z]}` — the native open-cell foam. The Voronoi
	/// edge graph is computed in-kernel (Bowyer–Watson, no scipy) and clipped to
	/// `[min, max]`; every strut gets the uniform `radius`.
	fn voronoi_lattice(&mut self, obj: &Map<String, Value>, path: &str) -> Result<Node, OpError> {
		let (min, max) = self.corner_box(obj, path)?;
		let radius = self.positive(obj, "radius", path)?;
		let seeds_arr = self
			.require(obj, "seeds", path)?
			.as_array()
			.ok_or_else(|| self.bad(path, "'seeds' must be an array of [x, y, z] generator points"))?;
		if seeds_arr.len() < 5 {
			return Err(self.bad(path, format!("'seeds' needs at least 5 points to form a foam, got {}", seeds_arr.len())));
		}
		if seeds_arr.len() > MAX_VORONOI_SEEDS {
			return Err(self.bad(path, format!("'seeds' has {} points, the cap is {MAX_VORONOI_SEEDS} (the in-kernel Delaunay is O(seeds²))", seeds_arr.len())));
		}
		let mut seeds = Vec::with_capacity(seeds_arr.len());
		for (i, v) in seeds_arr.iter().enumerate() {
			let arr = v.as_array().ok_or_else(|| self.bad(path, format!("'seeds'[{i}] must be an [x, y, z] array")))?;
			seeds.push(self.vec3_value(arr, &format!("seeds[{i}]"), path)?);
		}
		Ok(Node::primitive(VoronoiLattice::new(seeds, radius as f32, min, max)))
	}

	fn pipe(&mut self, obj: &Map<String, Value>, path: &str) -> Result<Node, OpError> {
		let pts_arr = self
			.require(obj, "path", path)?
			.as_array()
			.ok_or_else(|| self.bad(path, "'path' must be an array of [x, y, z] points"))?;
		if pts_arr.len() < 2 {
			return Err(self.bad(path, format!("'path' needs at least 2 points, got {}", pts_arr.len())));
		}
		if pts_arr.len() - 1 > MAX_PIPE_SEGMENTS {
			return Err(self.bad(path, format!("'path' has {} segments, the cap is {MAX_PIPE_SEGMENTS}", pts_arr.len() - 1)));
		}
		let mut pts = Vec::with_capacity(pts_arr.len());
		for (i, v) in pts_arr.iter().enumerate() {
			let arr = v.as_array().ok_or_else(|| self.bad(path, format!("'path'[{i}] must be an [x, y, z] array")))?;
			pts.push(self.vec3_value(arr, &format!("path[{i}]"), path)?);
		}
		let radii: Vec<f32> = match (obj.get("radii"), obj.get("radius")) {
			(Some(arr), None) => {
				let arr = arr.as_array().ok_or_else(|| self.bad(path, "'radii' must be an array of numbers"))?;
				if arr.len() != pts.len() {
					return Err(self.bad(path, format!("'radii' must have one entry per path point ({} points, {} radii)", pts.len(), arr.len())));
				}
				let mut out = Vec::with_capacity(arr.len());
				for (i, v) in arr.iter().enumerate() {
					let r = v.as_f64().unwrap_or(f64::NAN);
					if !(r.is_finite() && r > 0.0) {
						return Err(self.bad(path, format!("'radii'[{i}] must be a positive number, got {v}")));
					}
					out.push(r as f32);
				}
				out
			}
			(None, Some(_)) => vec![self.positive(obj, "radius", path)? as f32; pts.len()],
			_ => return Err(self.bad(path, "a pipe takes exactly one of 'radius' (constant) or 'radii' (one per path point)")),
		};
		Ok(Node::primitive(Pipe::new(pts, radii)))
	}

	fn combinator(&mut self, obj: &Map<String, Value>, op: &str, path: &str) -> Result<Node, OpError> {
		// Child-tree accessors.
		let two = |me: &mut Self| -> Result<(Node, Node), OpError> {
			let av = me.require(obj, "a", path)?;
			let a = me.node(av, &format!("{path}.a"))?;
			let bv = me.require(obj, "b", path)?;
			let b = me.node(bv, &format!("{path}.b"))?;
			Ok((a, b))
		};
		let one = |me: &mut Self| -> Result<Node, OpError> {
			let v = me.require(obj, "in", path)?;
			me.node(v, &format!("{path}.in"))
		};
		// Smooth/fillet/chamfer radius: finite and >= 0 (0 falls back to the hard op).
		let blend = |me: &Self, key: &str| -> Result<f32, OpError> {
			let k = me.num(obj, key, path)?;
			if k < 0.0 {
				return Err(me.bad(path, format!("field '{key}' must be >= 0, got {k}")));
			}
			Ok(k as f32)
		};

		match op {
			"union" => two(self).map(|(a, b)| a.union(b)),
			"intersection" => two(self).map(|(a, b)| a.intersection(b)),
			"difference" => two(self).map(|(a, b)| a.difference(b)),
			"smooth_union" => {
				let k = blend(self, "k")?;
				two(self).map(|(a, b)| a.smooth_union(b, k))
			}
			"smooth_intersection" => {
				let k = blend(self, "k")?;
				two(self).map(|(a, b)| a.smooth_intersection(b, k))
			}
			"smooth_difference" => {
				let k = blend(self, "k")?;
				two(self).map(|(a, b)| a.smooth_difference(b, k))
			}
			"fillet_union" => {
				let r = blend(self, "r")?;
				two(self).map(|(a, b)| fillet_union(a, b, r))
			}
			"fillet_difference" => {
				let r = blend(self, "r")?;
				two(self).map(|(a, b)| fillet_difference(a, b, r))
			}
			"chamfer_union" => {
				let r = blend(self, "r")?;
				two(self).map(|(a, b)| chamfer_union(a, b, r))
			}
			"chamfer_difference" => {
				let r = blend(self, "r")?;
				two(self).map(|(a, b)| chamfer_difference(a, b, r))
			}
			"displace" => {
				// Procedural surface texture: d′ = (d − amplitude·t(p)) / L′ with
				// t ∈ [0, 1] and L′ = 1 + |amplitude|·L_texture — the kernel divides
				// by the derived bound, so the ZERO SET (the geometry) is unchanged
				// and the emitted field stays ≤ 1-Lipschitz for a ≤ 1-Lipschitz child
				// (texture.rs derives each L_texture; narrow-band pruning stays sound).
				let amplitude = self.num(obj, "amplitude", path)?;
				let texture = self.texture(obj, path)?;
				one(self).map(|n| displaced(n, amplitude as f32, texture))
			}
			"offset" => {
				let t = self.num(obj, "t", path)?;
				one(self).map(|n| n.offset(t as f32))
			}
			"shell" => {
				let t = self.positive(obj, "t", path)?;
				one(self).map(|n| n.shell(t as f32))
			}
			"translate" => {
				let offset = self.vec3(obj, "offset", path)?;
				one(self).map(|n| n.translate(offset))
			}
			"rotate" => {
				let axis = self.axis(obj, "axis", path)?;
				let degrees = self.num(obj, "degrees", path)?;
				let center = match obj.get("center") {
					Some(_) => self.vec3(obj, "center", path)?,
					None => Vec3::ZERO,
				};
				let rot = Affine3A::from_axis_angle(axis, (degrees as f32).to_radians());
				let m = Affine3A::from_translation(center) * rot * Affine3A::from_translation(-center);
				one(self).map(|n| n.transform(m))
			}
			"scale" => {
				let factor = self.positive(obj, "factor", path)?;
				one(self).map(|n| n.scale(factor as f32))
			}
			"mirror" => {
				let point = self.vec3(obj, "point", path)?;
				let normal = self.axis(obj, "normal", path)?;
				one(self).map(|n| n.mirror(point, normal))
			}
			"linear_pattern" => {
				let step = self.vec3(obj, "step", path)?;
				let count = self.count(obj, "count", path, MAX_PATTERN_COUNT)?;
				one(self).map(|n| n.linear_pattern(step, count))
			}
			"circular_pattern" => {
				let center = self.vec3(obj, "center", path)?;
				let axis = self.axis(obj, "axis", path)?;
				let count = self.count(obj, "count", path, MAX_PATTERN_COUNT)?;
				let step_degrees = match obj.get("step_degrees") {
					Some(_) => self.num(obj, "step_degrees", path)?,
					None => 360.0 / count as f64,
				};
				one(self).map(|n| n.circular_pattern(center, axis, (step_degrees as f32).to_radians(), count))
			}
			"offset_by" => {
				let fv = self.require(obj, "field", path)?;
				let field = self.scalar_source(fv, &format!("{path}.field"))?;
				let max_abs = self.num(obj, "max_abs", path)?;
				if max_abs < 0.0 {
					return Err(self.bad(path, format!("'max_abs' must be >= 0 (the clamp on the field's offset), got {max_abs}")));
				}
				one(self).map(|n| n.offset_by(field, max_abs as f32))
			}
			"lerp" => {
				let fv = self.require(obj, "field", path)?;
				let field = self.scalar_source(fv, &format!("{path}.field"))?;
				two(self).map(|(a, b)| a.lerp(b, field))
			}
			other => Err(self.bad(path, format!("unknown combinator '{other}' — supported combinators: {COMBINATORS}"))),
		}
	}

	/// Parse the `texture` block of a `displace` combinator. Every parameter is
	/// validated HERE (positivity, [0, 1] ranges) so the kernel's own
	/// contract-violation panics are never reachable from JSON.
	fn texture(&self, obj: &Map<String, Value>, path: &str) -> Result<Texture, OpError> {
		let tv = self.require(obj, "texture", path)?;
		let tpath = format!("{path}.texture");
		let tobj = tv
			.as_object()
			.ok_or_else(|| self.bad(&tpath, "'texture' must be an object with a 'kind' field (knurl | stipple | noise)"))?;
		let unit = |me: &Self, key: &str, default: f64| -> Result<f64, OpError> {
			let v = match tobj.get(key) {
				None => default,
				Some(_) => me.num(tobj, key, &tpath)?,
			};
			if !(0.0..=1.0).contains(&v) {
				return Err(me.bad(&tpath, format!("'{key}' must be in [0, 1], got {v}")));
			}
			Ok(v)
		};
		match self.require(tobj, "kind", &tpath)?.as_str() {
			Some("knurl") => {
				// Crossed ±45° sinusoid ridges; peak-to-valley = amplitude·depth_frac/2
				// (the crossed gratings interfere — texture.rs derives the exact range).
				let pitch = self.positive(tobj, "pitch", &tpath)?;
				let depth_frac = unit(self, "depth_frac", 1.0)?;
				Ok(Texture::Knurl { pitch: pitch as f32, depth_frac: depth_frac as f32 })
			}
			Some("stipple") => {
				// Hash-scattered raised domes, `coverage` of the `cell` tiles occupied.
				let cell = self.positive(tobj, "cell", &tpath)?;
				let coverage = unit(self, "coverage", 0.5)?;
				Ok(Texture::Stipple { cell: cell as f32, coverage: coverage as f32 })
			}
			Some("noise") => {
				// Deterministic trilinear value-noise; `seed` picks the lattice values.
				let cell = self.positive(tobj, "cell", &tpath)?;
				let seed = match tobj.get("seed") {
					None => 0u32,
					Some(v) => {
						let n = v
							.as_u64()
							.filter(|n| *n <= u32::MAX as u64)
							.ok_or_else(|| self.bad(&tpath, format!("'seed' must be an integer in 0..=4294967295, got {v}")))?;
						n as u32
					}
				};
				Ok(Texture::Noise { cell: cell as f32, seed })
			}
			other => Err(self.bad(&tpath, format!("'kind' must be one of knurl|stipple|noise, got {other:?}"))),
		}
	}

	// --- Scalar-field sources (`offset_by` / `lerp` `field`) -------------------------

	/// The `field` of `offset_by` / `lerp`: either a scalar EXPRESSION (number /
	/// variable / math node — probed for finiteness over the meshing domain), or
	/// a sampled GRID source `{"grid": {…}}` (an `.npy` file through
	/// [`GridField`] — total, continuous, border-clamped and refused at load if
	/// any value is non-finite, so it needs no domain probe).
	fn scalar_source(&mut self, v: &Value, path: &str) -> Result<ScalarField, OpError> {
		if let Some(gv) = v.as_object().and_then(|o| o.get("grid")) {
			return self.grid_source(gv, &format!("{path}.grid"));
		}
		let expr = self.field(v, path)?;
		Ok(scalar_field(expr))
	}

	/// `{"grid": {"path": "field.npy", "origin": [x,y,z], "cell": h,
	/// "normalize": [lo, hi]?, "law": [at_zero, at_one]?}}` — simulation data as
	/// a grade source (`GridField::from_npy_bytes` → optional `normalized(lo,
	/// hi)` → `into_grade_law(at_zero, at_one)` or raw `into_scalar_field`).
	/// The `.npy` resolves confined under the input base (like `load_part`);
	/// an unreadable file is `io`, a malformed one `invalid_param` with the
	/// kernel's precise reason (dtype, shape, Fortran order, non-finite values).
	fn grid_source(&self, v: &Value, path: &str) -> Result<ScalarField, OpError> {
		let obj = v
			.as_object()
			.ok_or_else(|| self.bad(path, "'grid' must be an object {path, origin, cell, normalize?, law?}"))?;
		let file = self
			.require(obj, "path", path)?
			.as_str()
			.ok_or_else(|| self.bad(path, "'path' must be a string naming a .npy file"))?;
		let origin = self.vec3(obj, "origin", path)?;
		let cell = self.positive(obj, "cell", path)?;
		let resolved = crate::interp::resolve_input_path(self.op_id, self.input_base, file)?;
		let bytes = std::fs::read(&resolved).map_err(|e| OpError {
			kind: ErrorKind::Io,
			message: format!("op '{}': at {path}: cannot read '{}': {e}", self.op_id, resolved.display()),
		})?;
		let mut grid = GridField::from_npy_bytes(&bytes, origin, cell as f32).map_err(|e| self.bad(path, format!("'{file}': {e}")))?;
		if let Some(nv) = obj.get("normalize") {
			let (lo, hi) = self.num_pair(nv, "normalize", path)?;
			if hi <= lo {
				return Err(self.bad(path, format!("'normalize' needs lo < hi, got [{lo}, {hi}]")));
			}
			grid = grid.normalized(lo as f32, hi as f32);
		}
		match obj.get("law") {
			// The grade law: sampled value clamped to [0, 1], mapped 0 → at_zero,
			// 1 → at_one (mm; positive inflates) — the FEA-driven LinearGrade twin.
			Some(lv) => {
				let (at_zero, at_one) = self.num_pair(lv, "law", path)?;
				Ok(grid.into_grade_law(at_zero as f32, at_one as f32))
			}
			None => Ok(grid.into_scalar_field()),
		}
	}

	/// A `[a, b]` pair of finite numbers.
	fn num_pair(&self, v: &Value, what: &str, path: &str) -> Result<(f64, f64), OpError> {
		let arr = v
			.as_array()
			.filter(|a| a.len() == 2)
			.ok_or_else(|| self.bad(path, format!("'{what}' must be a 2-number array")))?;
		let mut out = [0.0f64; 2];
		for (i, x) in arr.iter().enumerate() {
			out[i] = x
				.as_f64()
				.filter(|n| n.is_finite())
				.ok_or_else(|| self.bad(path, format!("'{what}'[{i}] must be a finite number, got {x}")))?;
		}
		Ok((out[0], out[1]))
	}

	// --- The scalar-expression AST ------------------------------------------------

	/// Parse a scalar expression and register it for domain probing.
	fn field(&mut self, v: &Value, path: &str) -> Result<Arc<Expr>, OpError> {
		let expr = Arc::new(self.expr(v, path)?);
		self.fields.push(FieldProbe { path: path.to_string(), expr: Arc::clone(&expr), lipschitz: None });
		Ok(expr)
	}

	fn expr(&self, v: &Value, path: &str) -> Result<Expr, OpError> {
		match v {
			Value::Number(_) => {
				let n = v.as_f64().filter(|n| n.is_finite()).ok_or_else(|| self.bad(path, format!("constant {v} is not a finite number")))?;
				Ok(Expr::Const(n))
			}
			Value::String(s) => match s.as_str() {
				"x" => Ok(Expr::X),
				"y" => Ok(Expr::Y),
				"z" => Ok(Expr::Z),
				other => Err(self.bad(path, format!("unknown variable '{other}' — the only variables are \"x\", \"y\", \"z\""))),
			},
			Value::Object(obj) => {
				let op = self
					.require(obj, "op", path)?
					.as_str()
					.ok_or_else(|| self.bad(path, "scalar 'op' must be a string"))?;
				let sub = |me: &Self, key: &str| -> Result<Box<Expr>, OpError> {
					Ok(Box::new(me.expr(me.require(obj, key, path)?, &format!("{path}.{key}"))?))
				};
				match op {
					"add" => Ok(Expr::Add(sub(self, "a")?, sub(self, "b")?)),
					"sub" => Ok(Expr::Sub(sub(self, "a")?, sub(self, "b")?)),
					"mul" => Ok(Expr::Mul(sub(self, "a")?, sub(self, "b")?)),
					"div" => Ok(Expr::Div(sub(self, "a")?, sub(self, "b")?)),
					"min" => Ok(Expr::Min(sub(self, "a")?, sub(self, "b")?)),
					"max" => Ok(Expr::Max(sub(self, "a")?, sub(self, "b")?)),
					"mod" => Ok(Expr::Mod(sub(self, "a")?, sub(self, "b")?)),
					"atan2" => Ok(Expr::Atan2 { y: sub(self, "y")?, x: sub(self, "x")? }),
					"neg" => Ok(Expr::Neg(sub(self, "arg")?)),
					"abs" => Ok(Expr::Abs(sub(self, "arg")?)),
					"sqrt" => Ok(Expr::Sqrt(sub(self, "arg")?)),
					"sin" => Ok(Expr::Sin(sub(self, "arg")?)),
					"cos" => Ok(Expr::Cos(sub(self, "arg")?)),
					"clamp" => Ok(Expr::Clamp { value: sub(self, "value")?, lo: sub(self, "lo")?, hi: sub(self, "hi")? }),
					"length2" => Ok(Expr::Length2(sub(self, "a")?, sub(self, "b")?)),
					"length3" => Ok(Expr::Length3(sub(self, "a")?, sub(self, "b")?, sub(self, "c")?)),
					other => Err(self.bad(path, format!("unknown scalar op '{other}' — supported scalar ops: {SCALAR_OPS}"))),
				}
			}
			other => Err(self.bad(path, format!("a scalar expression is a number, one of \"x\"/\"y\"/\"z\", or an {{\"op\": ...}} object — got {other}"))),
		}
	}
}
