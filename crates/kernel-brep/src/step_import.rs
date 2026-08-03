// Copyright (c) LMCAD. Licensed under the MIT License.

//! ISO-10303-21 (STEP) **import**: parse the physical-file syntax into an entity
//! graph and reconstruct a B-rep [`Solid`].
//!
//! This is the read counterpart of [`crate::step_export`]. The parser is general
//! (it tokenises the full instance syntax — strings, enums, references, nested
//! lists and typed records), so it accepts STEP from any AP203/AP214 producer, not
//! only this kernel's own output. The public entry point [`import_step`] is
//! `Result`-returning so a caller (including an AI agent) gets a precise reason on
//! failure; nothing outside the matrix below is silently dropped.
//!
//! ## Support matrix
//!
//! | construct | handling |
//! |---|---|
//! | `PLANE`, `CYLINDRICAL/SPHERICAL/CONICAL/TOROIDAL_SURFACE` | exact analytic [`Surface`] tag |
//! | trimmed `B_SPLINE_SURFACE_WITH_KNOTS` face (incl. rational `_COMPLEX`) | tessellated **on the exact patch**: trim-loop vertices are Newton-projected into parameter space, the loops are triangulated there (monotone sweep for single rings, hole-bridging ear clip otherwise) and the interior is refined to the `PATCH_SAG_TOL` relative chordal tolerance — subject to the `PATCH_MIN_PITCH` area floor that pins the strip against an unsplittable trim chord (bounded residual sag there; the refinement's termination device) — with every interior vertex evaluated via [`NurbsSurface::point_at`]; trim chords are never split, so the weld with neighbour faces stays watertight. Facets carry their own exact `Plane` tags — the analytic [`Surface`] enum has no freeform variant, so the patch's NURBS identity is not on the [`Solid`] (it IS preserved by the [`import_step_freeform`] sidecar; exact patch reads also via [`import_bspline_surface`]) |
//! | **closed/periodic** B-spline face (`S` periodic across a domain end, verified by evaluation): a trim loop crossing the patch seam, a seam edge traversed twice (a real exporter's closed tube wall), or an untrimmed band bounded only by its two full-period rims | unwrapped into the **universal cover** (`unwrap_ring`, mirroring the analytic periodic-wall split): seam-crossing chords continue into the neighbouring period, slit traversals land one period apart and weld back on interning, two opposite full-period rims are bridged by a synthetic seam (`bridge_band_rings`); then the standard ear-clip + on-patch refinement. Caveat: each chord is unwrapped the SHORT way around, so a single trim chord deliberately spanning > half a period reads as a seam crossing |
//! | closed B-spline face whose loops wind the patch in any other combination (one winding rim, same-sense rims, winding holes), trim vertex off the patch | loud [`StepError::Unsupported`] |
//! | other surfaces (`SURFACE_OF_REVOLUTION`, offset, swept, …) | loud [`StepError::Unsupported`] |
//! | `FACE_OUTER_BOUND` + `FACE_BOUND` | multi-loop faces: planar and B-spline faces keep their inner (hole) loops; a curved ANALYTIC face with holes is refused loudly |
//! | `LINE` edges (or absent edge geometry) | the exact two-vertex chord |
//! | `CIRCLE` / `ELLIPSE` edges | sweep ≤ 90°: kept as the producer's chord (re-imports of this kernel's own faceted exports stay bit-exact); sweep > 90° through full rings: sampled at the `FULL_TURN_SEGMENTS` pitch (a one-edge full-circle cap becomes a closed 48-segment ring); segments carry the analytic [`Curve`] |
//! | `B_SPLINE_CURVE_WITH_KNOTS` edges (incl. rational `_COMPLEX`) | sampled over the knot domain at a curvature-adaptive pitch: `BSPLINE_EDGE_SEGMENTS` doubled (≤ `MAX_BSPLINE_EDGE_SEGMENTS`) while consecutive chords turn more than the conic ring pitch — so a closed full-circle B-spline rim gets 64 chords, a gentle freeform trim edge keeps 8 |
//! | `SURFACE_CURVE` / `SEAM_CURVE` edge wrappers | unwrapped to their 3-D curve |
//! | other edge geometry (`PARABOLA`, `TRIMMED_CURVE`, …) | loud [`StepError::Unsupported`] |
//! | periodic cylinder/cone face (seam edge + full-circle rims, e.g. a real exporter's cylinder wall) | split into chord-triangle facets on the exact surface via monotone parameter-strip triangulation — these are ruled in the axial direction, so the chords lie on the inscribed prism/frustum |
//! | periodic / pole-spanning sphere and torus regions (full sphere as one face, caps with or without a seam/pole vertex, bands between rims, full torus, torus bands) | resampled into a ring grid of chord facets ON the exact surface: boundary rings are reused verbatim (the weld), interior rings/pole fans are synthesized at the ring pitch (see `resample_periodic_region`) |
//! | general sub-periodic sphere/torus regions (≤ ~137° span, e.g. the recover pass's cubemap/quadrant chart faces) | triangulated on the exact surface: boundary verbatim, interior chord facets refined via the parameter chart (see `general_curved_region`) |
//! | partial-turn PERIODIC sphere/torus regions (a half-torus wall's full-turn tube rims, a pole-to-pole lune), rings off the grid phase, seam-free winding loops | loud [`StepError::Unsupported`] |
//! | `NEXT_ASSEMBLY_USAGE_OCCURRENCE` / `MAPPED_ITEM` assemblies | flattened component instances via [`import_step_assembly`] (names, per-part solids, accumulated placements) |
//!
//! Curved-face routing detail: a curved-tagged face whose tessellated boundary has
//! ≤ 4 vertices, or is planar **and** chord-close to its surface, imports as a single
//! chord facet — exactly this kernel's native representation of curved solids (and the
//! shape of its own exports). Only boundaries that cannot be one flat facet (a periodic
//! wall) are split.

use std::collections::HashMap;

use kernel_core::math::{DAffine3, DMat3, DVec2, DVec3};
use kernel_core::mesh::Mesh;
use kernel_core::orient2d;

use crate::geom::{perp_basis, Curve, Surface};
use crate::nurbs::{FreeformFace, NurbsCurve, NurbsSurface};
use crate::topo::{FaceLoops, Solid, VertexId};

/// Why a STEP import failed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StepError {
	/// The physical-file syntax could not be parsed.
	Parse(String),
	/// A referenced entity id is missing or has the wrong type.
	Reference(String),
	/// A geometry/topology construct this importer does not yet handle
	/// (e.g. a curved surface) was encountered.
	Unsupported(String),
	/// The reconstructed faces do not form a usable solid.
	Topology(String),
}

impl std::fmt::Display for StepError {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		match self {
			StepError::Parse(m) => write!(f, "STEP parse error: {m}"),
			StepError::Reference(m) => write!(f, "STEP reference error: {m}"),
			StepError::Unsupported(m) => write!(f, "unsupported STEP construct: {m}"),
			StepError::Topology(m) => write!(f, "STEP topology error: {m}"),
		}
	}
}

impl std::error::Error for StepError {}

/// A parsed STEP parameter value.
#[derive(Debug, Clone, PartialEq)]
enum Value {
	Real(f64),
	Int(i64),
	Str(String),
	/// An enumeration like `.T.` or `.PLANE.`, stored without the dots.
	Enum(String),
	/// A `#N` entity reference.
	Ref(u32),
	List(Vec<Value>),
	/// An inline typed record `NAME(args)`.
	Typed(String, Vec<Value>),
	/// `$` (unset) or `*` (derived).
	Null,
}

impl Value {
	fn as_ref(&self) -> Option<u32> {
		match self {
			Value::Ref(r) => Some(*r),
			_ => None,
		}
	}
	fn as_list(&self) -> Option<&[Value]> {
		match self {
			Value::List(v) => Some(v),
			_ => None,
		}
	}
	fn as_real(&self) -> Option<f64> {
		match self {
			Value::Real(r) => Some(*r),
			Value::Int(i) => Some(*i as f64),
			_ => None,
		}
	}
	fn as_int(&self) -> Option<i64> {
		match self {
			Value::Int(i) => Some(*i),
			_ => None,
		}
	}
}

/// One `#N = NAME(args);` instance.
struct Entity {
	name: String,
	args: Vec<Value>,
}

// --- Parser ------------------------------------------------------------------

/// Cursor over the bytes of a single instance body for recursive value parsing.
struct Cursor<'a> {
	s: &'a [u8],
	i: usize,
}

impl<'a> Cursor<'a> {
	fn new(s: &'a str) -> Self {
		Cursor { s: s.as_bytes(), i: 0 }
	}
	fn peek(&self) -> Option<u8> {
		self.s.get(self.i).copied()
	}
	fn skip_ws(&mut self) {
		while let Some(c) = self.peek() {
			if c.is_ascii_whitespace() {
				self.i += 1;
			} else {
				break;
			}
		}
	}

	/// Parse one value at the cursor.
	fn value(&mut self) -> Result<Value, StepError> {
		self.skip_ws();
		match self.peek() {
			None => Err(StepError::Parse("unexpected end of value".into())),
			Some(b'#') => {
				self.i += 1;
				let n = self.uint()?;
				Ok(Value::Ref(n))
			}
			Some(b'\'') => Ok(Value::Str(self.string()?)),
			Some(b'.') => Ok(Value::Enum(self.enumeration()?)),
			Some(b'(') => Ok(Value::List(self.list()?)),
			Some(b'$') | Some(b'*') => {
				self.i += 1;
				Ok(Value::Null)
			}
			Some(c) if c == b'+' || c == b'-' || c.is_ascii_digit() => self.number(),
			Some(c) if c.is_ascii_alphabetic() || c == b'_' => {
				let name = self.ident();
				self.skip_ws();
				if self.peek() == Some(b'(') {
					Ok(Value::Typed(name, self.list()?))
				} else {
					// A bare keyword constant (rare); treat as an enum-like token.
					Ok(Value::Enum(name))
				}
			}
			Some(c) => Err(StepError::Parse(format!("unexpected character '{}'", c as char))),
		}
	}

	/// Parse a parenthesised, comma-separated list (cursor on `(`).
	fn list(&mut self) -> Result<Vec<Value>, StepError> {
		debug_assert_eq!(self.peek(), Some(b'('));
		self.i += 1;
		let mut out = Vec::new();
		loop {
			self.skip_ws();
			match self.peek() {
				Some(b')') => {
					self.i += 1;
					return Ok(out);
				}
				None => return Err(StepError::Parse("unterminated list".into())),
				_ => {
					out.push(self.value()?);
					self.skip_ws();
					// `,` separates list items; `)` ends; anything else is the next
					// space-separated record of a complex instance `(A() B() C())`.
					match self.peek() {
						Some(b',') => self.i += 1,
						Some(b')') | None => {}
						_ => {}
					}
				}
			}
		}
	}

	fn uint(&mut self) -> Result<u32, StepError> {
		let start = self.i;
		while matches!(self.peek(), Some(c) if c.is_ascii_digit()) {
			self.i += 1;
		}
		if self.i == start {
			return Err(StepError::Parse("expected integer after '#'".into()));
		}
		std::str::from_utf8(&self.s[start..self.i])
			.ok()
			.and_then(|t| t.parse().ok())
			.ok_or_else(|| StepError::Parse("bad entity id".into()))
	}

	fn string(&mut self) -> Result<String, StepError> {
		debug_assert_eq!(self.peek(), Some(b'\''));
		self.i += 1;
		let mut out = String::new();
		loop {
			match self.peek() {
				None => return Err(StepError::Parse("unterminated string".into())),
				Some(b'\'') => {
					// `''` is an escaped single quote.
					if self.s.get(self.i + 1) == Some(&b'\'') {
						out.push('\'');
						self.i += 2;
					} else {
						self.i += 1;
						return Ok(out);
					}
				}
				Some(c) => {
					out.push(c as char);
					self.i += 1;
				}
			}
		}
	}

	fn enumeration(&mut self) -> Result<String, StepError> {
		debug_assert_eq!(self.peek(), Some(b'.'));
		self.i += 1;
		let start = self.i;
		while matches!(self.peek(), Some(c) if c != b'.') {
			self.i += 1;
		}
		if self.peek() != Some(b'.') {
			return Err(StepError::Parse("unterminated enumeration".into()));
		}
		let s = std::str::from_utf8(&self.s[start..self.i]).unwrap_or("").to_string();
		self.i += 1;
		Ok(s)
	}

	fn ident(&mut self) -> String {
		let start = self.i;
		while matches!(self.peek(), Some(c) if c.is_ascii_alphanumeric() || c == b'_') {
			self.i += 1;
		}
		std::str::from_utf8(&self.s[start..self.i]).unwrap_or("").to_string()
	}

	fn number(&mut self) -> Result<Value, StepError> {
		let start = self.i;
		let mut real = false;
		if matches!(self.peek(), Some(b'+') | Some(b'-')) {
			self.i += 1;
		}
		while let Some(c) = self.peek() {
			match c {
				b'0'..=b'9' => self.i += 1,
				b'.' => {
					real = true;
					self.i += 1;
				}
				b'e' | b'E' => {
					real = true;
					self.i += 1;
					if matches!(self.peek(), Some(b'+') | Some(b'-')) {
						self.i += 1;
					}
				}
				_ => break,
			}
		}
		let t = std::str::from_utf8(&self.s[start..self.i]).unwrap_or("");
		if real {
			t.parse::<f64>().map(Value::Real).map_err(|_| StepError::Parse(format!("bad real '{t}'")))
		} else {
			t.parse::<i64>().map(Value::Int).map_err(|_| StepError::Parse(format!("bad integer '{t}'")))
		}
	}
}

/// Split the file into top-level `;`-terminated statements, ignoring `;` inside
/// `'…'` strings and `/* … */` comments.
fn statements(text: &str) -> Vec<String> {
	let b = text.as_bytes();
	let mut out = Vec::new();
	let mut cur = String::new();
	let mut i = 0;
	let mut in_str = false;
	while i < b.len() {
		let c = b[i];
		if in_str {
			cur.push(c as char);
			if c == b'\'' {
				if b.get(i + 1) == Some(&b'\'') {
					cur.push('\'');
					i += 2;
					continue;
				}
				in_str = false;
			}
			i += 1;
		} else if c == b'/' && b.get(i + 1) == Some(&b'*') {
			// Skip a block comment.
			i += 2;
			while i + 1 < b.len() && !(b[i] == b'*' && b[i + 1] == b'/') {
				i += 1;
			}
			i += 2;
		} else if c == b'\'' {
			in_str = true;
			cur.push('\'');
			i += 1;
		} else if c == b';' {
			out.push(cur.trim().to_string());
			cur.clear();
			i += 1;
		} else {
			cur.push(c as char);
			i += 1;
		}
	}
	out
}

/// Parse the whole file into an instance map (`#N → Entity`).
fn parse(text: &str) -> Result<HashMap<u32, Entity>, StepError> {
	let mut map = HashMap::new();
	for stmt in statements(text) {
		// Only `#N = …` instance statements carry geometry.
		let Some(rest) = stmt.strip_prefix('#') else { continue };
		let Some(eq) = rest.find('=') else { continue };
		let id: u32 = rest[..eq].trim().parse().map_err(|_| StepError::Parse(format!("bad id in '{stmt}'")))?;
		let body = rest[eq + 1..].trim();
		let mut cur = Cursor::new(body);
		let parsed = cur.value().map_err(|e| match e {
			StepError::Parse(m) => StepError::Parse(format!("{m} in `{body}`")),
			other => other,
		})?;
		match parsed {
			Value::Typed(name, args) => {
				map.insert(id, Entity { name, args });
			}
			// A complex instance `#N=(A(..)B(..))` parses as a list of typed records;
			// keep the records under a synthetic name so lookups by sub-type still work.
			Value::List(items) => {
				map.insert(id, Entity { name: "_COMPLEX".into(), args: items });
			}
			_ => {} // non-entity assignment — ignore
		}
	}
	Ok(map)
}

// --- Reconstruction ----------------------------------------------------------

struct Importer<'a> {
	ents: &'a HashMap<u32, Entity>,
}

impl<'a> Importer<'a> {
	fn get(&self, id: u32) -> Result<&Entity, StepError> {
		self.ents.get(&id).ok_or_else(|| StepError::Reference(format!("missing entity #{id}")))
	}

	/// Resolve a `CARTESIAN_POINT` reference to a 3-D position.
	fn point(&self, id: u32) -> Result<DVec3, StepError> {
		let e = self.get(id)?;
		if e.name != "CARTESIAN_POINT" {
			return Err(StepError::Reference(format!("#{id} is {}, expected CARTESIAN_POINT", e.name)));
		}
		let coords = e
			.args
			.iter()
			.find_map(Value::as_list)
			.ok_or_else(|| StepError::Parse(format!("#{id} CARTESIAN_POINT has no coordinate list")))?;
		let c: Vec<f64> = coords.iter().filter_map(Value::as_real).collect();
		if c.len() < 3 {
			return Err(StepError::Parse(format!("#{id} CARTESIAN_POINT needs 3 coordinates")));
		}
		Ok(DVec3::new(c[0], c[1], c[2]))
	}

	/// Resolve a `DIRECTION` reference to a vector.
	fn direction(&self, id: u32) -> Result<DVec3, StepError> {
		let e = self.get(id)?;
		if e.name != "DIRECTION" {
			return Err(StepError::Reference(format!("#{id} is {}, expected DIRECTION", e.name)));
		}
		let coords = e
			.args
			.iter()
			.find_map(Value::as_list)
			.ok_or_else(|| StepError::Parse(format!("#{id} DIRECTION has no component list")))?;
		let c: Vec<f64> = coords.iter().filter_map(Value::as_real).collect();
		if c.len() < 3 {
			return Err(StepError::Parse(format!("#{id} DIRECTION needs 3 components")));
		}
		Ok(DVec3::new(c[0], c[1], c[2]))
	}

	/// Resolve a `VERTEX_POINT` reference to its position.
	fn vertex(&self, id: u32) -> Result<DVec3, StepError> {
		let e = self.get(id)?;
		if e.name != "VERTEX_POINT" {
			return Err(StepError::Reference(format!("#{id} is {}, expected VERTEX_POINT", e.name)));
		}
		let cp = e.args.iter().find_map(Value::as_ref).ok_or_else(|| StepError::Parse(format!("#{id} VERTEX_POINT has no point")))?;
		self.point(cp)
	}

	/// Resolve a face's supporting [`Surface`] (plane and the four analytic quadrics).
	fn surface(&self, id: u32) -> Result<Surface, StepError> {
		let e = self.get(id)?;
		let placement = || {
			e.args.iter().find_map(Value::as_ref).ok_or_else(|| StepError::Parse(format!("#{id} {} has no placement", e.name)))
		};
		// Trailing scalar parameters (radius, semi-angle, …) after the placement.
		let reals: Vec<f64> = e.args.iter().filter_map(Value::as_real).collect();
		let real = |k: usize| reals.get(k).copied().ok_or_else(|| StepError::Parse(format!("#{id} {} missing scalar {k}", e.name)));
		match e.name.as_str() {
			"PLANE" => {
				let (origin, normal, _) = self.placement(placement()?)?;
				Ok(Surface::Plane { origin, normal })
			}
			"CYLINDRICAL_SURFACE" => {
				let (origin, axis, _) = self.placement(placement()?)?;
				Ok(Surface::Cylinder { origin, axis, radius: real(0)? })
			}
			"SPHERICAL_SURFACE" => {
				let (center, _, _) = self.placement(placement()?)?;
				Ok(Surface::Sphere { center, radius: real(0)? })
			}
			"CONICAL_SURFACE" => {
				// STEP places the cone at a reference plane (radius `r` there, half
				// angle `a`); the apex is back along −axis where the radius vanishes.
				let (location, axis, _) = self.placement(placement()?)?;
				let (r, half_angle) = (real(0)?, real(1)?);
				let apex = location - axis * (r / half_angle.tan());
				Ok(Surface::Cone { apex, axis, half_angle })
			}
			"TOROIDAL_SURFACE" => {
				let (center, axis, _) = self.placement(placement()?)?;
				Ok(Surface::Torus { center, axis, major: real(0)?, minor: real(1)? })
			}
			other => Err(StepError::Unsupported(format!("surface #{id} of type {other}"))),
		}
	}

	/// Parse a `B_SPLINE_SURFACE_WITH_KNOTS` entity into a [`NurbsSurface`] (the
	/// non-rational case; unit weights). Control points come from the
	/// `CARTESIAN_POINT` grid, and the full knot vectors are reconstructed from the
	/// distinct-knot + multiplicity lists. Evaluation and tessellation are then
	/// provided by [`NurbsSurface`] — the reading half of NURBS interchange.
	fn bspline_surface(&self, id: u32) -> Result<NurbsSurface, StepError> {
		let e = self.get(id)?;
		// Accept a plain B_SPLINE_SURFACE_WITH_KNOTS entity OR a _COMPLEX (rational) instance, which
		// splits its data across B_SPLINE_SURFACE (degrees + control grid), B_SPLINE_SURFACE_WITH_KNOTS
		// (knots + multiplicities) and RATIONAL_B_SPLINE_SURFACE (a weight grid). Reading the weight
		// grid is what makes an imported cylinder/sphere/conic surface (which CAD exporters encode as a
		// rational B-spline) geometrically exact instead of silently de-rationalised to unit weights.
		let (field_args, rational_weights): (Vec<Value>, Option<Vec<Vec<f64>>>) = match e.name.as_str() {
			"B_SPLINE_SURFACE_WITH_KNOTS" => (e.args.clone(), None),
			"_COMPLEX" => {
				let mut a = Vec::new();
				for sub in ["B_SPLINE_SURFACE", "B_SPLINE_SURFACE_WITH_KNOTS"] {
					if let Some(p) = complex_part(&e.args, sub) {
						a.extend_from_slice(p);
					}
				}
				if a.is_empty() {
					return Err(StepError::Reference(format!("#{id} _COMPLEX has no B_SPLINE_SURFACE record")));
				}
				let w = complex_part(&e.args, "RATIONAL_B_SPLINE_SURFACE")
					.and_then(|p| {
						p.iter().find_map(|v| match v {
							Value::List(rows) if matches!(rows.first(), Some(Value::List(_))) => Some(rows),
							_ => None,
						})
					})
					.map(|rows| {
						rows.iter()
							.filter_map(|r| r.as_list().map(|cells| cells.iter().filter_map(Value::as_real).collect::<Vec<f64>>()))
							.collect()
					});
				(a, w)
			}
			other => return Err(StepError::Reference(format!("#{id} is {other}, expected B_SPLINE_SURFACE_WITH_KNOTS"))),
		};
		let args = &field_args;
		// The two top-level integer scalars are the u- and v-degrees (multiplicities and
		// control references live inside nested lists, never at the top level).
		let degs: Vec<usize> = args.iter().filter_map(|v| v.as_int().map(|i| i.max(0) as usize)).collect();
		if degs.len() < 2 {
			return Err(StepError::Parse(format!("#{id} B_SPLINE_SURFACE_WITH_KNOTS missing degrees")));
		}
		let (deg_u, deg_v) = (degs[0], degs[1]);
		// Control grid: the first list whose elements are themselves lists (of point refs).
		let grid = args
			.iter()
			.find_map(|v| match v {
				Value::List(rows) if matches!(rows.first(), Some(Value::List(_))) => Some(rows),
				_ => None,
			})
			.ok_or_else(|| StepError::Parse(format!("#{id} B_SPLINE_SURFACE_WITH_KNOTS has no control grid")))?;
		let mut control: Vec<Vec<DVec3>> = Vec::with_capacity(grid.len());
		for row in grid {
			let cells = row.as_list().ok_or_else(|| StepError::Parse(format!("#{id} control row is not a list")))?;
			let mut pts = Vec::with_capacity(cells.len());
			for c in cells {
				let cp = c.as_ref().ok_or_else(|| StepError::Parse(format!("#{id} control cell is not a point reference")))?;
				pts.push(self.point(cp)?);
			}
			control.push(pts);
		}
		// The remaining flat lists are, in order, the u- and v-multiplicities (all-Int)
		// and the u- and v-knots (all-Real); the control grid (lists/refs) is skipped.
		let mut mults: Vec<Vec<i64>> = Vec::new();
		let mut knots: Vec<Vec<f64>> = Vec::new();
		for v in args {
			let Value::List(items) = v else { continue };
			if items.is_empty() || items.iter().any(|x| matches!(x, Value::List(_) | Value::Ref(_))) {
				continue;
			}
			if items.iter().all(|x| matches!(x, Value::Int(_))) {
				mults.push(items.iter().filter_map(Value::as_int).collect());
			} else if items.iter().all(|x| matches!(x, Value::Real(_) | Value::Int(_))) {
				knots.push(items.iter().filter_map(Value::as_real).collect());
			}
		}
		if mults.len() < 2 || knots.len() < 2 {
			return Err(StepError::Parse(format!("#{id} B_SPLINE_SURFACE_WITH_KNOTS missing knot/multiplicity lists")));
		}
		let knots_u = expand_knots(&knots[0], &mults[0]);
		let knots_v = expand_knots(&knots[1], &mults[1]);
		let weights: Vec<Vec<f64>> = rational_weights.unwrap_or_else(|| control.iter().map(|r| vec![1.0; r.len()]).collect());
		NurbsSurface::new(deg_u, deg_v, knots_u, knots_v, control, weights)
			.ok_or_else(|| StepError::Topology(format!("#{id} B_SPLINE_SURFACE_WITH_KNOTS has inconsistent dimensions")))
	}

	/// Parse a `B_SPLINE_CURVE_WITH_KNOTS` entity into a [`NurbsCurve`] (non-rational;
	/// unit weights). The control points are a flat `CARTESIAN_POINT` reference list and
	/// the full knot vector is reconstructed from the distinct-knot + multiplicity lists.
	fn bspline_curve(&self, id: u32) -> Result<NurbsCurve, StepError> {
		let e = self.get(id)?;
		// Accept a plain B_SPLINE_CURVE_WITH_KNOTS entity OR a _COMPLEX (rational) instance, which
		// splits its data across B_SPLINE_CURVE (degree + control points), B_SPLINE_CURVE_WITH_KNOTS
		// (knots + multiplicities) and RATIONAL_B_SPLINE_CURVE (per-control-point weights). Reading
		// the weights is what makes an imported circle/conic (which CAD exporters encode as a
		// rational B-spline) geometrically exact instead of silently de-rationalised to unit weights.
		let (field_args, rational_weights): (Vec<Value>, Option<Vec<f64>>) = match e.name.as_str() {
			"B_SPLINE_CURVE_WITH_KNOTS" => (e.args.clone(), None),
			"_COMPLEX" => {
				let mut a = Vec::new();
				for sub in ["B_SPLINE_CURVE", "B_SPLINE_CURVE_WITH_KNOTS"] {
					if let Some(p) = complex_part(&e.args, sub) {
						a.extend_from_slice(p);
					}
				}
				if a.is_empty() {
					return Err(StepError::Reference(format!("#{id} _COMPLEX has no B_SPLINE_CURVE record")));
				}
				let w = complex_part(&e.args, "RATIONAL_B_SPLINE_CURVE")
					.and_then(|p| p.iter().find_map(Value::as_list))
					.map(|w| w.iter().filter_map(Value::as_real).collect());
				(a, w)
			}
			other => return Err(StepError::Reference(format!("#{id} is {other}, expected B_SPLINE_CURVE_WITH_KNOTS"))),
		};
		let args = &field_args;
		let degree = args.iter().find_map(Value::as_int).map(|i| i.max(0) as usize).ok_or_else(|| StepError::Parse(format!("#{id} B_SPLINE_CURVE_WITH_KNOTS missing degree")))?;
		// Control points: the first list whose elements are point references.
		let refs = args
			.iter()
			.find_map(|v| match v {
				Value::List(items) if matches!(items.first(), Some(Value::Ref(_))) => Some(items),
				_ => None,
			})
			.ok_or_else(|| StepError::Parse(format!("#{id} B_SPLINE_CURVE_WITH_KNOTS has no control list")))?;
		let mut control = Vec::with_capacity(refs.len());
		for c in refs {
			let cp = c.as_ref().ok_or_else(|| StepError::Parse(format!("#{id} control cell is not a point reference")))?;
			control.push(self.point(cp)?);
		}
		// The flat Int list is the multiplicities; the flat Real list, the distinct knots.
		let mut mults: Option<Vec<i64>> = None;
		let mut knots: Option<Vec<f64>> = None;
		for v in args {
			let Value::List(items) = v else { continue };
			if items.is_empty() || items.iter().any(|x| matches!(x, Value::Ref(_) | Value::List(_))) {
				continue;
			}
			if items.iter().all(|x| matches!(x, Value::Int(_))) {
				mults.get_or_insert_with(|| items.iter().filter_map(Value::as_int).collect());
			} else if items.iter().all(|x| matches!(x, Value::Real(_) | Value::Int(_))) {
				knots.get_or_insert_with(|| items.iter().filter_map(Value::as_real).collect());
			}
		}
		let mults = mults.ok_or_else(|| StepError::Parse(format!("#{id} B_SPLINE_CURVE_WITH_KNOTS missing multiplicities")))?;
		let knots = knots.ok_or_else(|| StepError::Parse(format!("#{id} B_SPLINE_CURVE_WITH_KNOTS missing knots")))?;
		let weights = rational_weights.unwrap_or_else(|| vec![1.0; control.len()]);
		NurbsCurve::new(degree, expand_knots(&knots, &mults), control, weights)
			.ok_or_else(|| StepError::Topology(format!("#{id} B_SPLINE_CURVE_WITH_KNOTS has inconsistent dimensions")))
	}

	/// `AXIS2_PLACEMENT_3D` → (location, axis/normal, ref direction).
	fn placement(&self, id: u32) -> Result<(DVec3, DVec3, DVec3), StepError> {
		let e = self.get(id)?;
		if e.name != "AXIS2_PLACEMENT_3D" {
			return Err(StepError::Reference(format!("#{id} is {}, expected AXIS2_PLACEMENT_3D", e.name)));
		}
		let refs: Vec<u32> = e.args.iter().filter_map(Value::as_ref).collect();
		if refs.is_empty() {
			return Err(StepError::Parse(format!("#{id} AXIS2_PLACEMENT_3D has no location")));
		}
		let origin = self.point(refs[0])?;
		// Axis and ref direction are optional in the schema; default sensibly.
		let normal = refs.get(1).map(|&r| self.direction(r)).transpose()?.unwrap_or(DVec3::Z);
		let refdir = refs.get(2).map(|&r| self.direction(r)).transpose()?.unwrap_or(DVec3::X);
		Ok((origin, normal.normalize_or_zero(), refdir))
	}

	/// Orthonormal frame of an `AXIS2_PLACEMENT_3D`: `(location, unit axis, unit x, unit y)`
	/// with `x` the reference direction re-orthogonalised against the axis (per ISO
	/// 10303-42) and `y = axis × x` — the frame conic parameter angles are measured in.
	fn frame(&self, id: u32) -> Result<(DVec3, DVec3, DVec3, DVec3), StepError> {
		let (origin, axis, refdir) = self.placement(id)?;
		let axis = if axis.length_squared() > 1e-20 { axis } else { DVec3::Z };
		let r = refdir - axis * refdir.dot(axis);
		let x = if r.length_squared() > 1e-20 { r.normalize() } else { perp_basis(axis).0 };
		Ok((origin, axis, x, axis.cross(x)))
	}

	/// Canonical polyline of an `EDGE_CURVE` in the edge's own v_start→v_end direction,
	/// plus the analytic [`Curve`] its segments lie on (for conic geometry). `LINE`
	/// edges (and absent geometry) are the exact two-vertex chord. `CIRCLE`/`ELLIPSE`
	/// edges keep their chord up to a 90° sweep (the producer's own granularity, which
	/// keeps re-imports of this kernel's faceted exports exact) and are sampled at the
	/// `FULL_TURN_SEGMENTS` pitch beyond that — a full ring (identical start/end
	/// vertex) becomes a closed 48-segment ring. B-spline edges are sampled over their
	/// knot domain; `SURFACE_CURVE`/`SEAM_CURVE` wrappers are unwrapped to their 3-D
	/// curve. Anything else is a loud [`StepError::Unsupported`]. Endpoints are always
	/// the exact `VERTEX_POINT` positions so shared edges intern identically.
	fn edge_polyline(&self, ec_id: u32) -> Result<(Vec<DVec3>, Option<Curve>), StepError> {
		let ec = self.get(ec_id)?;
		if ec.name != "EDGE_CURVE" {
			return Err(StepError::Reference(format!("#{ec_id} is {}, expected EDGE_CURVE", ec.name)));
		}
		// EDGE_CURVE('', #v_start, #v_end, #curve, same_sense)
		let refs: Vec<u32> = ec.args.iter().filter_map(Value::as_ref).collect();
		if refs.len() < 2 {
			return Err(StepError::Parse(format!("#{ec_id} EDGE_CURVE needs two vertices")));
		}
		let start = self.vertex(refs[0])?;
		let end = self.vertex(refs[1])?;
		let same_sense = last_enum(ec).map(|s| s == "T").unwrap_or(true);
		let Some(&geom_id) = refs.get(2) else {
			return Ok((vec![start, end], None)); // no geometry reference: the straight chord
		};
		let mut gid = geom_id;
		let mut g = self.get(gid)?;
		// AP214 wraps an edge's 3-D geometry: SURFACE_CURVE('', #curve, (pcurves), repr).
		if g.name == "SURFACE_CURVE" || g.name == "SEAM_CURVE" {
			gid = g.args.iter().find_map(Value::as_ref).ok_or_else(|| StepError::Parse(format!("#{gid} {} has no 3-D curve", g.name)))?;
			g = self.get(gid)?;
		}
		match g.name.as_str() {
			"LINE" => Ok((vec![start, end], None)),
			"CIRCLE" => {
				// CIRCLE('', #placement, radius)
				let placement = g.args.iter().find_map(Value::as_ref).ok_or_else(|| StepError::Parse(format!("#{gid} CIRCLE has no placement")))?;
				let (center, axis, x, y) = self.frame(placement)?;
				let radius = g
					.args
					.iter()
					.rev()
					.find_map(Value::as_real)
					.ok_or_else(|| StepError::Parse(format!("#{gid} CIRCLE has no radius")))?;
				let angle = |p: DVec3| (p - center).dot(y).atan2((p - center).dot(x));
				let sweep = edge_sweep(angle(start), angle(end), same_sense, ec_id)?;
				let eval = |t: f64| center + (x * t.cos() + y * t.sin()) * radius;
				Ok((sample_arc(start, end, angle(start), sweep, eval), Some(Curve::Circle { center, normal: axis, radius })))
			}
			"ELLIPSE" => {
				// ELLIPSE('', #placement, semi_axis_1, semi_axis_2): semi_axis_1 lies along
				// the placement's reference direction, semi_axis_2 along axis × ref.
				let placement = g.args.iter().find_map(Value::as_ref).ok_or_else(|| StepError::Parse(format!("#{gid} ELLIPSE has no placement")))?;
				let (center, axis, x, y) = self.frame(placement)?;
				let semis: Vec<f64> = g.args.iter().filter_map(Value::as_real).collect();
				if semis.len() < 2 || !(semis[0] > 0.0 && semis[1] > 0.0) {
					return Err(StepError::Parse(format!("#{gid} ELLIPSE needs two positive semi-axes")));
				}
				let (a1, a2) = (semis[0], semis[1]);
				let angle = |p: DVec3| ((p - center).dot(y) / a2).atan2((p - center).dot(x) / a1);
				let sweep = edge_sweep(angle(start), angle(end), same_sense, ec_id)?;
				let eval = |t: f64| center + x * (a1 * t.cos()) + y * (a2 * t.sin());
				// [`Curve::Ellipse`] wants `u` = the semi-MAJOR direction with a ≥ b; the
				// swapped frame parameterises the same point set (a quarter-phase shift).
				let curve = if a1 >= a2 {
					Curve::Ellipse { center, normal: axis, u: x, a: a1, b: a2 }
				} else {
					Curve::Ellipse { center, normal: axis, u: y, a: a2, b: a1 }
				};
				Ok((sample_arc(start, end, angle(start), sweep, eval), Some(curve)))
			}
			name if name == "B_SPLINE_CURVE_WITH_KNOTS"
				|| (name == "_COMPLEX" && complex_part(&g.args, "B_SPLINE_CURVE_WITH_KNOTS").is_some()) =>
			{
				let c = self.bspline_curve(gid)?;
				let (lo, hi) = c.domain();
				// Curvature-honest segment count: start at the historical
				// BSPLINE_EDGE_SEGMENTS granularity and double while any consecutive
				// chord pair still turns more than the conic ring pitch (2π /
				// FULL_TURN_SEGMENTS), capped at MAX_BSPLINE_EDGE_SEGMENTS. Gentle
				// freeform trim edges keep their 8 segments; a closed full-circle
				// B-spline rim (start == end vertex, which a fixed 8 would polygonise
				// to a 10%-deficit octagon) or a long sweeping arc is refined until
				// its chords match the fidelity of imported conic rings.
				let mut n = BSPLINE_EDGE_SEGMENTS;
				while n < MAX_BSPLINE_EDGE_SEGMENTS && max_chord_turn(&c, n) > std::f64::consts::TAU / FULL_TURN_SEGMENTS as f64 {
					n *= 2;
				}
				let mut pts = Vec::with_capacity(n + 1);
				pts.push(start);
				for k in 1..n {
					let f = k as f64 / n as f64;
					// Sample in the edge's direction of travel over the knot domain.
					let t = if same_sense { lo + (hi - lo) * f } else { hi - (hi - lo) * f };
					pts.push(c.point_at(t));
				}
				pts.push(end);
				Ok((pts, None))
			}
			other => Err(StepError::Unsupported(format!(
				"edge #{ec_id} geometry #{gid} of type {other} — importable edge curves are LINE, CIRCLE, ELLIPSE and B_SPLINE_CURVE_WITH_KNOTS"
			))),
		}
	}

	/// Tessellated, ordered boundary of an `EDGE_LOOP`: each `ORIENTED_EDGE` contributes
	/// its edge's canonical polyline (forward for `.T.`, reversed for `.F.`) minus the
	/// closing point, which the next edge supplies. Returns the boundary positions and,
	/// parallel to them, the analytic [`Curve`] each segment `points[i] → points[i+1 mod n]`
	/// lies on. Polylines are cached per `EDGE_CURVE` id, so the two faces sharing an
	/// edge get bit-identical points — the twin-pairing prerequisite.
	fn loop_boundary(
		&self,
		edge_loop: u32,
		cache: &mut HashMap<u32, (Vec<DVec3>, Option<Curve>)>,
	) -> Result<(Vec<DVec3>, Vec<Option<Curve>>), StepError> {
		let e = self.get(edge_loop)?;
		if e.name != "EDGE_LOOP" {
			return Err(StepError::Reference(format!("#{edge_loop} is {}, expected EDGE_LOOP", e.name)));
		}
		let oriented = e.args.iter().find_map(Value::as_list).ok_or_else(|| StepError::Parse(format!("#{edge_loop} EDGE_LOOP has no edge list")))?;
		let mut boundary = Vec::with_capacity(oriented.len());
		let mut curves = Vec::with_capacity(oriented.len());
		for oe in oriented {
			let oe_id = oe.as_ref().ok_or_else(|| StepError::Parse("EDGE_LOOP item is not a reference".into()))?;
			let oe_ent = self.get(oe_id)?;
			if oe_ent.name != "ORIENTED_EDGE" {
				return Err(StepError::Reference(format!("#{oe_id} is {}, expected ORIENTED_EDGE", oe_ent.name)));
			}
			// ORIENTED_EDGE('', *, *, #edge_curve, .T./.F.)
			let ec_id = oe_ent.args.iter().find_map(Value::as_ref).ok_or_else(|| StepError::Parse(format!("#{oe_id} ORIENTED_EDGE has no edge")))?;
			let oe_sense = last_enum(oe_ent).map(|s| s == "T").unwrap_or(true);
			if let std::collections::hash_map::Entry::Vacant(slot) = cache.entry(ec_id) {
				slot.insert(self.edge_polyline(ec_id)?);
			}
			let (pts, curve) = &cache[&ec_id];
			let m = pts.len();
			for k in 0..m - 1 {
				boundary.push(if oe_sense { pts[k] } else { pts[m - 1 - k] });
				curves.push(*curve);
			}
		}
		Ok((boundary, curves))
	}

	/// The placement axis (local +Z `DIRECTION`) of an analytic surface entity — the
	/// datum a periodic sphere face is unwrapped about. The placement is the surface's
	/// first reference; a missing axis defaults to +Z per the schema.
	fn surface_axis(&self, id: u32) -> Result<DVec3, StepError> {
		let e = self.get(id)?;
		let placement = e
			.args
			.iter()
			.find_map(Value::as_ref)
			.ok_or_else(|| StepError::Parse(format!("#{id} {} has no placement", e.name)))?;
		let (_, axis, _) = self.placement(placement)?;
		Ok(if axis.length_squared() > 1e-20 { axis.normalize() } else { DVec3::Z })
	}

	/// `(ADVANCED_FACE id, reversed)` pairs of one shell, resolving `ORIENTED_CLOSED_SHELL`
	/// wrappers: a `.F.` wrapper logically reverses every contained face's loops (real
	/// exporters use it for the void shells of a `BREP_WITH_VOIDS` and for mirrored
	/// instances). Faces keep the shell's stored order.
	fn shell_faces(&self, shell_id: u32, flip: bool) -> Result<ShellFaces, StepError> {
		let e = self.get(shell_id)?;
		match e.name.as_str() {
			"CLOSED_SHELL" | "OPEN_SHELL" => {
				let list = e
					.args
					.iter()
					.find_map(Value::as_list)
					.ok_or_else(|| StepError::Parse(format!("#{shell_id} {} has no face list", e.name)))?;
				let mut out = Vec::with_capacity(list.len());
				for f in list {
					let fid = f.as_ref().ok_or_else(|| StepError::Parse(format!("#{shell_id} shell face is not a reference")))?;
					out.push((fid, flip));
				}
				Ok(out)
			}
			"ORIENTED_CLOSED_SHELL" => {
				// ORIENTED_CLOSED_SHELL('', *, #shell, .T./.F.)
				let inner = e
					.args
					.iter()
					.find_map(Value::as_ref)
					.ok_or_else(|| StepError::Parse(format!("#{shell_id} ORIENTED_CLOSED_SHELL has no shell")))?;
				let same = last_enum(e).map(|s| s == "T").unwrap_or(true);
				self.shell_faces(inner, flip ^ !same)
			}
			other => Err(StepError::Unsupported(format!("shell #{shell_id} of type {other}"))),
		}
	}

	/// The `(face id, reversed)` sets of every `MANIFOLD_SOLID_BREP` / `BREP_WITH_VOIDS`
	/// in ascending entity-id order (deterministic across runs), keyed by the brep id.
	/// Empty when the file carries no solid-model entities (a bare-face fragment).
	fn brep_face_sets(&self) -> Result<Vec<(u32, ShellFaces)>, StepError> {
		let mut brep_ids: Vec<u32> = self
			.ents
			.iter()
			.filter(|(_, e)| e.name == "MANIFOLD_SOLID_BREP" || e.name == "BREP_WITH_VOIDS")
			.map(|(&id, _)| id)
			.collect();
		brep_ids.sort_unstable();
		let mut out = Vec::with_capacity(brep_ids.len());
		for id in brep_ids {
			let e = self.get(id)?;
			// MANIFOLD_SOLID_BREP('', #outer) — BREP_WITH_VOIDS('', #outer, (#voids…)),
			// each void an ORIENTED_CLOSED_SHELL (normals already point INTO the material,
			// i.e. out of the cavity, when its flag is honoured).
			let outer = e
				.args
				.iter()
				.find_map(Value::as_ref)
				.ok_or_else(|| StepError::Parse(format!("#{id} {} has no outer shell", e.name)))?;
			let mut faces = self.shell_faces(outer, false)?;
			if e.name == "BREP_WITH_VOIDS" {
				let voids = e
					.args
					.iter()
					.find_map(Value::as_list)
					.ok_or_else(|| StepError::Parse(format!("#{id} BREP_WITH_VOIDS has no void list")))?;
				for v in voids {
					let vid = v.as_ref().ok_or_else(|| StepError::Parse(format!("#{id} void shell is not a reference")))?;
					faces.extend(self.shell_faces(vid, false)?);
				}
			}
			out.push((id, faces));
		}
		Ok(out)
	}

	/// Every `ADVANCED_FACE` id in ascending order — the fallback face set for files
	/// (e.g. snippets) that carry bare faces without `MANIFOLD_SOLID_BREP` structure.
	fn all_face_ids(&self) -> ShellFaces {
		let mut ids: Vec<u32> = self
			.ents
			.iter()
			.filter(|(_, e)| e.name == "ADVANCED_FACE")
			.map(|(&id, _)| id)
			.collect();
		ids.sort_unstable();
		ids.into_iter().map(|id| (id, false)).collect()
	}
}

/// Segments a full 2π conic ring is tessellated into on import (~7.5° per chord).
const FULL_TURN_SEGMENTS: usize = 48;

/// Largest conic-arc sweep (radians) imported as a single chord between its two
/// vertices. Up to here the producer's own edge granularity is respected — which also
/// keeps a re-import of this kernel's faceted exports bit-identical (round-trip volume
/// preserved to 1e-6). Beyond it (through full 2π rings, whose endpoints alone cannot
/// describe the boundary at all) the arc is subdivided at the `FULL_TURN_SEGMENTS`
/// pitch so the boundary is geometrically faithful.
const MAX_CHORD_SWEEP: f64 = std::f64::consts::FRAC_PI_2;

/// Baseline segments a B-spline edge curve is sampled into across its knot domain
/// (doubled adaptively up to [`MAX_BSPLINE_EDGE_SEGMENTS`] while consecutive chords
/// turn by more than the conic ring pitch — see `edge_polyline`).
const BSPLINE_EDGE_SEGMENTS: usize = 8;

/// Hard cap on adaptive B-spline edge sampling (a full rational circle terminates
/// at 64 segments — 5.6° per chord, finer than the 48-segment conic pitch).
const MAX_BSPLINE_EDGE_SEGMENTS: usize = 64;

/// Largest turn angle (radians) between consecutive chords of an `n`-segment uniform
/// parameter sampling of a B-spline curve — the curvature witness that drives the
/// adaptive edge pitch. Near-zero chords (repeated control points) are skipped.
fn max_chord_turn(c: &NurbsCurve, n: usize) -> f64 {
	let (lo, hi) = c.domain();
	let pts: Vec<DVec3> = (0..=n).map(|k| c.point_at(lo + (hi - lo) * k as f64 / n as f64)).collect();
	let scale = 1.0 + pts.iter().map(|p| p.length()).fold(0.0_f64, f64::max);
	let mut chords: Vec<DVec3> = Vec::with_capacity(n);
	for w in pts.windows(2) {
		let d = w[1] - w[0];
		if d.length() > 1e-12 * scale {
			chords.push(d.normalize());
		}
	}
	let mut turn = 0.0_f64;
	for w in chords.windows(2) {
		turn = turn.max(w[0].dot(w[1]).clamp(-1.0, 1.0).acos());
	}
	turn
}

/// The signed sweep of a conic edge from angle `t0` to `t1`: in `(0, 2π]` when the edge
/// follows the curve's parameterisation (`same_sense`), in `[−2π, 0)` against it.
/// Identical endpoint angles mean a FULL ring (sweep ±2π), per the STEP convention
/// that a closed edge reuses one vertex.
fn edge_sweep(t0: f64, t1: f64, same_sense: bool, ec_id: u32) -> Result<f64, StepError> {
	use std::f64::consts::TAU;
	if !t0.is_finite() || !t1.is_finite() {
		return Err(StepError::Parse(format!("edge #{ec_id} has non-finite arc endpoint angles")));
	}
	let mut sweep = t1 - t0; // atan2 outputs keep this within [−2π, 2π]
	if same_sense {
		while sweep <= 1e-9 {
			sweep += TAU;
		}
	} else {
		while sweep >= -1e-9 {
			sweep -= TAU;
		}
	}
	Ok(sweep)
}

/// Sample a conic arc from `start` to `end` (kept as the exact vertex positions),
/// sweeping `sweep` radians from parameter `t0` (negative = against the curve's
/// parameterisation). One chord up to `MAX_CHORD_SWEEP`, else the `FULL_TURN_SEGMENTS`
/// pitch.
fn sample_arc(start: DVec3, end: DVec3, t0: f64, sweep: f64, eval: impl Fn(f64) -> DVec3) -> Vec<DVec3> {
	use std::f64::consts::TAU;
	let n = if sweep.abs() <= MAX_CHORD_SWEEP {
		1
	} else {
		(sweep.abs() / (TAU / FULL_TURN_SEGMENTS as f64)).ceil() as usize
	};
	let mut pts = Vec::with_capacity(n + 1);
	pts.push(start);
	for k in 1..n {
		pts.push(eval(t0 + sweep * k as f64 / n as f64));
	}
	pts.push(end);
	pts
}

/// Last enumeration argument of an entity (the trailing `.T./.F.` flag).
fn last_enum(e: &Entity) -> Option<String> {
	e.args.iter().rev().find_map(|v| match v {
		Value::Enum(s) => Some(s.clone()),
		_ => None,
	})
}

/// Bit-exact key for de-duplicating coincident vertex positions.
type PosKey = (u64, u64, u64);

/// Bit-exact key for de-duplicating coincident vertex positions.
fn pos_key(p: DVec3) -> PosKey {
	(p.x.to_bits(), p.y.to_bits(), p.z.to_bits())
}

/// `(face id, loop-reversal flag)` pairs of one shell.
type ShellFaces = Vec<(u32, bool)>;

/// Expand a STEP `(distinct knots, multiplicities)` pair into a full knot vector,
/// repeating each distinct knot by its multiplicity.
fn expand_knots(distinct: &[f64], mults: &[i64]) -> Vec<f64> {
	let mut k = Vec::new();
	for (&val, &m) in distinct.iter().zip(mults) {
		for _ in 0..m.max(0) {
			k.push(val);
		}
	}
	k
}

/// The argument list of the named sub-record inside a `_COMPLEX` instance's args, if present
/// (e.g. the `RATIONAL_B_SPLINE_CURVE` weights record within a rational B-spline complex).
fn complex_part<'a>(args: &'a [Value], name: &str) -> Option<&'a [Value]> {
	args.iter().find_map(|v| match v {
		Value::Typed(n, a) if n == name => Some(a.as_slice()),
		_ => None,
	})
}

/// Import the first `B_SPLINE_SURFACE_WITH_KNOTS` (non-rational NURBS) surface in a
/// STEP file into a [`NurbsSurface`] — the reading half of NURBS interchange. The
/// result can be evaluated ([`NurbsSurface::point_at`]) and tessellated
/// ([`NurbsSurface::tessellate`]). Returns [`StepError::Unsupported`] if the file
/// has no such entity.
pub fn import_bspline_surface(text: &str) -> Result<NurbsSurface, StepError> {
	let ents = parse(text)?;
	let imp = Importer { ents: &ents };
	let id = ents
		.iter()
		.find(|(_, e)| {
			e.name == "B_SPLINE_SURFACE_WITH_KNOTS"
				|| (e.name == "_COMPLEX" && complex_part(&e.args, "B_SPLINE_SURFACE_WITH_KNOTS").is_some())
		})
		.map(|(&id, _)| id)
		.ok_or_else(|| StepError::Unsupported("no B_SPLINE_SURFACE_WITH_KNOTS entity".into()))?;
	imp.bspline_surface(id)
}

/// Import the first `B_SPLINE_CURVE_WITH_KNOTS` (non-rational NURBS) curve in a STEP
/// file into a [`NurbsCurve`]. Companion to [`import_bspline_surface`] for trim/edge
/// curves. Returns [`StepError::Unsupported`] if the file has no such entity.
pub fn import_bspline_curve(text: &str) -> Result<NurbsCurve, StepError> {
	let ents = parse(text)?;
	let imp = Importer { ents: &ents };
	let id = ents
		.iter()
		.find(|(_, e)| {
			e.name == "B_SPLINE_CURVE_WITH_KNOTS"
				|| (e.name == "_COMPLEX" && complex_part(&e.args, "B_SPLINE_CURVE_WITH_KNOTS").is_some())
		})
		.map(|(&id, _)| id)
		.ok_or_else(|| StepError::Unsupported("no B_SPLINE_CURVE_WITH_KNOTS entity".into()))?;
	imp.bspline_curve(id)
}

/// Import the first `B_SPLINE_SURFACE_WITH_KNOTS` in a STEP file and tessellate it
/// into a [`Mesh`] at an `nu × nv` sample grid — the end-to-end NURBS read path: a
/// freeform STEP surface becomes printable/renderable triangles in one call. The
/// patch is sampled over its full parameter domain (untrimmed).
pub fn import_bspline_mesh(text: &str, nu: usize, nv: usize) -> Result<Mesh, StepError> {
	Ok(import_bspline_surface(text)?.tessellate(nu, nv))
}

/// Remove consecutive duplicate positions (and a duplicated wrap-around point) from a
/// boundary ring — zero-length segments from degenerate edges in the input.
fn dedup_ring(pts: &mut Vec<DVec3>) {
	pts.dedup_by(|a, b| pos_key(*a) == pos_key(*b));
	while pts.len() > 1 && pos_key(pts[0]) == pos_key(pts[pts.len() - 1]) {
		pts.pop();
	}
}

/// Newell area vector of a polygon (winding-following, UNnormalised — its length is
/// twice the enclosed area, so a periodic slit loop yields a near-zero vector).
fn newell_vector(pts: &[DVec3]) -> DVec3 {
	let mut nv = DVec3::ZERO;
	let len = pts.len();
	for i in 0..len {
		let c = pts[i];
		let d = pts[(i + 1) % len];
		nv.x += (c.y - d.y) * (c.z + d.z);
		nv.y += (c.z - d.z) * (c.x + d.x);
		nv.z += (c.x - d.x) * (c.y + d.y);
	}
	nv
}

/// Whether an entity is a B-spline surface (plain `B_SPLINE_SURFACE_WITH_KNOTS` or a
/// rational `_COMPLEX` instance carrying that record).
fn is_bspline_surface(e: &Entity) -> bool {
	e.name == "B_SPLINE_SURFACE_WITH_KNOTS"
		|| (e.name == "_COMPLEX" && complex_part(&e.args, "B_SPLINE_SURFACE_WITH_KNOTS").is_some())
}

/// Whether a curved-tagged boundary with more than four vertices is a flat CHORD FACET
/// of its surface: planar to tolerance AND spanning at most `MAX_CHORD_SWEEP` of the
/// surface's angular extent. Boolean-recovered bands are such facets (coplanar corners,
/// small sagitta, e.g. a clipped bore wall whose straight cuts added collinear
/// vertices); a real exporter's pole-spanning cap is NOT — its rim is planar but spans
/// the full turn — and must be treated as a curved region instead.
fn is_chord_facet(pts: &[DVec3], surface: &Surface) -> bool {
	let len = pts.len();
	let centroid = pts.iter().copied().sum::<DVec3>() / len as f64;
	let scale = pts.iter().map(|p| (*p - centroid).length()).fold(0.0_f64, f64::max);
	if scale <= 0.0 {
		return true; // a coincident point cluster is degenerate but trivially flat
	}
	let nv = newell_vector(pts);
	// A periodic (slit) loop encloses ~zero projected area relative to its extent.
	if nv.length() < 1e-8 * scale * scale {
		return false;
	}
	let n = nv.normalize();
	pts.iter().all(|p| (*p - centroid).dot(n).abs() < 1e-7 * scale) && boundary_angular_span(pts, surface) <= MAX_CHORD_SWEEP
}

/// Greatest pairwise angle (radians) a boundary spans on its surface: about the axis
/// for cylinder/cone, between radius directions for a sphere, and the larger of the
/// about-axis and around-tube spans for a torus.
fn boundary_angular_span(pts: &[DVec3], surface: &Surface) -> f64 {
	let about_axis = |origin: DVec3, axis: DVec3| -> Vec<DVec3> {
		pts.iter()
			.filter_map(|p| {
				let d = *p - origin;
				let radial = d - axis * d.dot(axis);
				(radial.length_squared() > 1e-18).then(|| radial.normalize())
			})
			.collect()
	};
	match *surface {
		Surface::Plane { .. } => 0.0,
		Surface::Cylinder { origin, axis, .. } => max_pairwise_angle(&about_axis(origin, axis.normalize_or_zero())),
		Surface::Cone { apex, axis, .. } => max_pairwise_angle(&about_axis(apex, axis.normalize_or_zero())),
		Surface::Sphere { center, .. } => {
			let dirs: Vec<DVec3> = pts
				.iter()
				.filter_map(|p| {
					let d = *p - center;
					(d.length_squared() > 1e-18).then(|| d.normalize())
				})
				.collect();
			max_pairwise_angle(&dirs)
		}
		Surface::Torus { center, axis, major, .. } => {
			let axis = axis.normalize_or_zero();
			let tube: Vec<DVec3> = pts
				.iter()
				.filter_map(|p| {
					let d = *p - center;
					let h = d.dot(axis);
					let rho = (d - axis * h).length();
					// Around-tube direction embedded in a fixed 2-D frame.
					let t = DVec3::new(rho - major, h, 0.0);
					(t.length_squared() > 1e-18).then(|| t.normalize())
				})
				.collect();
			max_pairwise_angle(&about_axis(center, axis)).max(max_pairwise_angle(&tube))
		}
	}
}

/// Largest angle between any two of `dirs` (unit vectors).
fn max_pairwise_angle(dirs: &[DVec3]) -> f64 {
	let mut min_dot = 1.0_f64;
	for i in 0..dirs.len() {
		for j in i + 1..dirs.len() {
			min_dot = min_dot.min(dirs[i].dot(dirs[j]));
		}
	}
	min_dot.clamp(-1.0, 1.0).acos()
}

/// Wrap an angle difference into `(−π, π]`.
fn wrap_half_turn(d: f64) -> f64 {
	use std::f64::consts::{PI, TAU};
	let mut d = d % TAU;
	if d <= -PI {
		d += TAU;
	} else if d > PI {
		d -= TAU;
	}
	d
}

/// Split a curved face whose tessellated boundary cannot be one chord facet — a
/// periodic cylinder/cone wall (full-circle rims + a seam edge, the shape real
/// exporters emit) — into chord-triangle facets on its surface, returned as index
/// triples into `pts` wound like the input loop.
///
/// The boundary is unwrapped into the surface's `(angle·radius, axial)` parameter
/// strip: the seam's two copies land one period apart, and a cone-apex point (where
/// the angle is undefined) interpolates between its neighbours. The resulting
/// u-monotone polygon is triangulated sweep-line style. Cylinder and cone are RULED
/// along the axial direction, so the chord triangles lie on the inscribed prism/
/// frustum — geometrically faithful at the ring pitch. Sphere/torus regions would
/// need pole/bi-periodic interior sampling and are refused loudly instead.
fn split_periodic_face(pts: &[DVec3], surface: &Surface, fid: u32) -> Result<Vec<[usize; 3]>, StepError> {
	use std::f64::consts::TAU;
	let (origin, axis) = match *surface {
		Surface::Cylinder { origin, axis, .. } => (origin, axis),
		Surface::Cone { apex, axis, .. } => (apex, axis),
		Surface::Sphere { .. } | Surface::Torus { .. } => {
			return Err(StepError::Unsupported(format!(
				"ADVANCED_FACE #{fid}: a sphere/torus face spanning more than a chord facet (e.g. a pole-spanning cap) is not importable — re-export with faceted curved faces"
			)));
		}
		Surface::Plane { .. } => {
			return Err(StepError::Topology(format!(
				"ADVANCED_FACE #{fid}: planar face reached the periodic splitter"
			)));
		}
	};
	let axis = axis.normalize_or_zero();
	let (e1, e2) = perp_basis(axis);
	let n = pts.len();
	let mut theta = vec![0.0_f64; n];
	let mut defined = vec![false; n];
	let mut axial = vec![0.0_f64; n];
	let mut r_rep = 0.0_f64;
	for (i, p) in pts.iter().enumerate() {
		let d = *p - origin;
		axial[i] = d.dot(axis);
		let radial = d - axis * axial[i];
		let r = radial.length();
		r_rep = r_rep.max(r);
		if r > 1e-9 * (1.0 + d.length()) {
			theta[i] = radial.dot(e2).atan2(radial.dot(e1));
			defined[i] = true;
		}
	}
	if r_rep <= 0.0 {
		return Err(StepError::Topology(format!(
			"ADVANCED_FACE #{fid}: face boundary collapses onto its surface axis"
		)));
	}
	// Unwrap the angle along the loop: each defined step stays within half a turn of
	// the previous defined value, so the seam's second copy lands a full period away.
	let mut u = vec![0.0_f64; n];
	let mut first: Option<usize> = None;
	let mut prev: Option<usize> = None;
	for i in 0..n {
		if !defined[i] {
			continue;
		}
		u[i] = match prev {
			None => theta[i],
			Some(j) => u[j] + wrap_half_turn(theta[i] - theta[j]),
		};
		first.get_or_insert(i);
		prev = Some(i);
	}
	let (Some(first), Some(last)) = (first, prev) else {
		return Err(StepError::Topology(format!(
			"ADVANCED_FACE #{fid}: face boundary has no off-axis points"
		)));
	};
	if defined.iter().all(|&d| d) {
		// A fully defined loop must close in angle (winding 0): one that comes back a
		// full turn off has no seam edge and bounds no disk-like parameter region.
		let closure = u[last] + wrap_half_turn(theta[first] - theta[last]) - u[first];
		if closure.abs() > TAU / 4.0 {
			return Err(StepError::Unsupported(format!(
				"ADVANCED_FACE #{fid}: the face boundary winds around its periodic surface without a seam edge and cannot bound a parameter region"
			)));
		}
	} else {
		// Interpolate undefined (apex) angles linearly between their flanking defined
		// neighbours; across the loop start the chain continues from the previous
		// unwrapped value rather than restarting at the datum.
		for i in 0..n {
			if defined[i] {
				continue;
			}
			let (mut a, mut da) = ((i + n - 1) % n, 1usize);
			while !defined[a] {
				a = (a + n - 1) % n;
				da += 1;
			}
			let (mut b, mut db) = ((i + 1) % n, 1usize);
			while !defined[b] {
				b = (b + 1) % n;
				db += 1;
			}
			let ub = u[a] + wrap_half_turn(theta[b] - theta[a]);
			u[i] = u[a] + (ub - u[a]) * da as f64 / (da + db) as f64;
		}
	}
	// Strip coordinates: angle scaled to arc length (conditioning), axial as-is.
	let uv: Vec<DVec2> = (0..n).map(|i| DVec2::new(u[i] * r_rep, axial[i])).collect();
	triangulate_monotone(&uv).or_else(|_| triangulate_earclip(&uv))
		.map_err(|m| StepError::Unsupported(format!("ADVANCED_FACE #{fid}: cannot triangulate the unwrapped boundary ({m})")))
}

/// Largest angular EXTENT (radians) a region may subtend and still be read by
/// the general chart triangulation ([`general_curved_region`]), per surface
/// family — the injective domain of the chart the refinement works in:
///
/// - **sphere** → gnomonic about the region's mean direction, injective on the
///   open hemisphere; ~137° leaves margin (a recover-pass cubemap sextant
///   spans ~110°). A full sphere / pole-spanning cap reads π and is refused.
/// - **cylinder / cone / torus** → the unrolled angle chart, injective below a
///   FULL turn; 5.6 rad (~321°) accepts every sub-periodic region (a half-wrap
///   chart face reads π) and refuses a periodic wall (~2π), which belongs to
///   the seam-aware [`split_periodic_face`] / [`resample_periodic_region`].
///
/// The extent is [`crate::recover::angular_span`] (2π − the largest gap), NOT
/// the max pairwise angle: the latter saturates at π, so it cannot tell a
/// half-wrap sector from a full periodic wall.
fn general_region_span_max(surface: &Surface) -> f64 {
	match surface {
		Surface::Sphere { .. } => 2.4,
		_ => 5.6,
	}
}

/// **General sub-periodic curved region** import path: triangulate the
/// boundary in the surface's parameter chart with interior refinement
/// ([`crate::tessellate::refine_curved_ring`]) — chord facets ON the exact
/// surface, wound like the input loop, volume-faithful to the refinement
/// tolerance and consuming the boundary VERBATIM (so neighbouring faces stay
/// welded). This is the read path for the recover pass's merged chart faces
/// (half-wrap cylinder/cone sectors, sphere cubemap sextants, torus quadrant
/// grids) whose polyline bound is neither a flat chord facet nor a lat-long
/// ring grid — including the jagged ones a mesher-derived solid produces, for
/// which the seam-aware splitters have no valid parameterisation.
///
/// Returns `(extra interior points, triangles)` in
/// [`resample_periodic_region`]'s convention; `None` (caller keeps the periodic
/// path) when the region is a full periodic wrap
/// ([`general_region_span_max`]) or cannot be charted.
fn general_curved_region(pts: &[DVec3], surface: &Surface) -> Option<(Vec<DVec3>, Vec<[usize; 3]>)> {
	// Read a face back exactly the way the tessellator writes it: this is the
	// SAME merged-face test `tessellate` uses to decide whether a curved ring
	// gets interior refinement. An ordinary chord-facet band (a boolean-cut bore
	// wall) answers `false` and keeps the flat-chord path, which is what holds
	// this kernel's own export → import round-trip exact; only a genuinely
	// merged chart face is re-triangulated on its surface.
	let nv = newell_vector(pts).normalize_or_zero();
	if !crate::tessellate::merged_curved_ring(pts, surface, nv) {
		return None;
	}
	if crate::recover::angular_span(surface, pts) > general_region_span_max(surface) {
		return None;
	}
	// A torus is periodic in TWO directions: the azimuth check above cannot see
	// a wall that wraps the tube completely (a half-torus wall spans only π
	// about the axis but a full 2π around the tube). Guard it explicitly, so
	// such a wall stays the loud `Unsupported` it has always been.
	if let Surface::Torus { center, axis, major, .. } = *surface {
		let axis = axis.normalize_or_zero();
		let mut psi: Vec<f64> = pts
			.iter()
			.filter_map(|&p| {
				let d = p - center;
				let h = d.dot(axis);
				let rho = (d - axis * h).length();
				let t = DVec2::new(rho - major, h);
				(t.length_squared() > 1e-18).then(|| t.y.atan2(t.x))
			})
			.collect();
		if psi.len() < 2 {
			return None;
		}
		psi.sort_by(f64::total_cmp);
		let mut max_gap = std::f64::consts::TAU - (psi[psi.len() - 1] - psi[0]);
		for w in psi.windows(2) {
			max_gap = max_gap.max(w[1] - w[0]);
		}
		if std::f64::consts::TAU - max_gap > general_region_span_max(surface) {
			return None;
		}
	}
	let (all, tris, outward) = crate::tessellate::refine_curved_ring(pts, surface)?;
	// Wind each facet like the input loop: the loop's Newell vector agrees with
	// the (sign-corrected) surface normal for an outward-wound boundary, and the
	// per-triangle reference comes from the chart centroid (never degenerate).
	let nv = newell_vector(pts);
	let sigma = if pts.iter().map(|&p| surface.normal_at(p).dot(nv)).sum::<f64>() < 0.0 { -1.0 } else { 1.0 };
	let oriented = tris
		.into_iter()
		.enumerate()
		.map(|(i, [a, b, c])| {
			let geo = (all[b] - all[a]).cross(all[c] - all[a]);
			if geo.dot(outward[i] * sigma) < 0.0 {
				[a, c, b]
			} else {
				[a, b, c]
			}
		})
		.collect();
	Some((all[pts.len()..].to_vec(), oriented))
}

/// Angular tolerance (radians) for grouping boundary samples into rings/levels and
/// matching ring sample longitudes to grid columns.
const RING_ANG_TOL: f64 = 1e-6;

/// The two periodic coordinates of a sphere/torus about a chosen `axis`:
/// `phi` is the longitude about the axis (periodic in both surfaces) and `level`
/// is the latitude (sphere, `[-π/2, π/2]`, poles at the ends) or the tube angle
/// (torus, periodic). Rings of constant `level` are the circles real exporters
/// bound these faces with.
struct PeriodicFrame {
	center: DVec3,
	axis: DVec3,
	e1: DVec3,
	e2: DVec3,
	kind: FrameKind,
}

enum FrameKind {
	Sphere { radius: f64 },
	Torus { major: f64, minor: f64 },
}

impl PeriodicFrame {
	fn new(surface: &Surface, axis: DVec3) -> Option<Self> {
		let axis = axis.normalize_or_zero();
		if axis.length_squared() < 0.5 {
			return None;
		}
		let (e1, e2) = perp_basis(axis);
		match *surface {
			Surface::Sphere { center, radius } => Some(Self { center, axis, e1, e2, kind: FrameKind::Sphere { radius } }),
			Surface::Torus { center, major, minor, .. } => Some(Self { center, axis, e1, e2, kind: FrameKind::Torus { major, minor } }),
			_ => None,
		}
	}

	/// `(phi, level, phi_defined)` of a surface point. `phi` is undefined on the
	/// axis (a sphere pole).
	fn coords(&self, p: DVec3) -> (f64, f64, bool) {
		let d = p - self.center;
		let h = d.dot(self.axis);
		let radial = d - self.axis * h;
		let rho = radial.length();
		let defined = rho > 1e-9 * (1.0 + d.length());
		let phi = if defined { radial.dot(self.e2).atan2(radial.dot(self.e1)) } else { 0.0 };
		let level = match self.kind {
			FrameKind::Sphere { .. } => h.atan2(rho),
			FrameKind::Torus { major, .. } => h.atan2(rho - major),
		};
		(phi, level, defined)
	}

	/// Exact surface point at `(level, phi)`.
	fn eval(&self, level: f64, phi: f64) -> DVec3 {
		let u = self.e1 * phi.cos() + self.e2 * phi.sin();
		match self.kind {
			FrameKind::Sphere { radius } => self.center + (u * level.cos() + self.axis * level.sin()) * radius,
			FrameKind::Torus { major, minor } => {
				self.center + u * (major + minor * level.cos()) + self.axis * (minor * level.sin())
			}
		}
	}

	/// Unit surface direction of increasing `level` at `(level, phi)`.
	fn level_dir(&self, level: f64, phi: f64) -> DVec3 {
		let u = self.e1 * phi.cos() + self.e2 * phi.sin();
		(self.axis * level.cos() - u * level.sin()).normalize_or_zero()
	}

	/// Outward surface normal at `(level, phi)`.
	fn normal(&self, level: f64, phi: f64) -> DVec3 {
		let u = self.e1 * phi.cos() + self.e2 * phi.sin();
		u * level.cos() + self.axis * level.sin()
	}

	/// Whether `level` itself wraps the full turn (the torus tube direction) rather
	/// than terminating at poles (the sphere latitude).
	fn level_cyclic(&self) -> bool {
		matches!(self.kind, FrameKind::Torus { .. })
	}

	fn pole(&self, north: bool) -> DVec3 {
		match self.kind {
			FrameKind::Sphere { radius } => self.center + self.axis * if north { radius } else { -radius },
			FrameKind::Torus { .. } => unreachable!("a torus has no poles"),
		}
	}
}

/// A full-turn ring of boundary samples at one `level`: `cols[k]` is the boundary
/// point index at longitude `phi0 + k·2π/n`. `slit` marks a ring every sample of
/// which appears ≥ 2× in the loop (a seam ring of a fully periodic face — the loop
/// traverses it in both directions, so it carries no orientation information).
struct BoundaryRing {
	level: f64,
	cols: Vec<usize>,
	slit: bool,
}

/// One row of the resampled grid: a full ring of point handles, or a single pole.
/// Handles `< pts.len()` are boundary indices; the rest index the extras.
enum GridRow {
	Ring(Vec<usize>),
	Pole(usize),
}

/// Resample a periodic / pole-spanning **sphere or torus** face region into a ring
/// grid of chord facets ON the exact surface — the import route for the curved-face
/// shapes real exporters emit:
///
/// - a full sphere as ONE face (seam meridian + two pole vertices);
/// - a spherical cap (rim circle, with or without a seam-to-the-pole excursion);
/// - a sphere band between two rim circles (a ball with two flats);
/// - a full torus as ONE face (equator + tube seams);
/// - a torus band between two rims (the classic fillet ring).
///
/// The boundary is decomposed about the surface axis into full-turn **rings**,
/// **poles** and **seam (slit) points** (positions the loop traverses twice — both
/// sides belong to this face, so after exact-position interning they pair
/// internally and need no facet edge). Ring rows reuse the boundary samples
/// verbatim (the weld with neighbour faces), interior rows are synthesized ON the
/// exact surface at the ring pitch, and pole rows fan to the exact pole vertex.
/// Facet orientation follows the loop's traversal of the first non-slit ring; a
/// fully periodic face (slits only) has a zero-area loop, so orientation falls back
/// to the face's `same_sense` flag against the analytic outward normal.
///
/// Returns `(extra interior points, triangles)`: triangle indices `< pts.len()`
/// reference the input boundary, the rest index the extras. Anything that does not
/// decompose into rings/poles/slits (e.g. a half-torus wall bounded by tube
/// circles, misaligned ring phases, a lune) is a loud [`StepError::Unsupported`].
fn resample_periodic_region(
	pts: &[DVec3],
	surface: &Surface,
	axis: DVec3,
	same_sense: bool,
	fid: u32,
) -> Result<(Vec<DVec3>, Vec<[usize; 3]>), StepError> {
	// Candidate unwrap axes: the surface placement axis and, for a sphere, the rim
	// plane normal (a cap whose rim is tilted against the placement axis is still a
	// ring about its OWN axis through the center).
	let mut candidates = vec![axis];
	if matches!(surface, Surface::Sphere { .. }) {
		let nv = newell_vector(pts);
		if nv.length_squared() > 1e-16 {
			let n = nv.normalize();
			if n.dot(axis.normalize_or_zero()).abs() < 1.0 - 1e-9 {
				candidates.push(n);
			}
		}
	}
	let mut last_err = String::from("no usable unwrap axis");
	for a in candidates {
		let Some(frame) = PeriodicFrame::new(surface, a) else { continue };
		match try_resample_grid(pts, &frame, same_sense) {
			Ok(out) => return Ok(out),
			Err(m) => last_err = m,
		}
	}
	Err(StepError::Unsupported(format!(
		"ADVANCED_FACE #{fid}: periodic sphere/torus region not importable — {last_err}"
	)))
}

/// The grid construction behind [`resample_periodic_region`] for one axis candidate.
fn try_resample_grid(pts: &[DVec3], frame: &PeriodicFrame, same_sense: bool) -> Result<(Vec<DVec3>, Vec<[usize; 3]>), String> {
	use std::f64::consts::{FRAC_PI_2, TAU};
	let n_pts = pts.len();

	// Per-point coordinates and per-position loop multiplicity.
	let coords: Vec<(f64, f64, bool)> = pts.iter().map(|&p| frame.coords(p)).collect();
	let mut occ: HashMap<PosKey, u32> = HashMap::new();
	for &p in pts {
		*occ.entry(pos_key(p)).or_insert(0) += 1;
	}
	let multiplicity = |i: usize| occ[&pos_key(pts[i])];

	// Distinct positions (first index wins), separated into poles and ring candidates.
	let mut seen: HashMap<PosKey, usize> = HashMap::new();
	let mut distinct: Vec<usize> = Vec::new();
	for (i, &p) in pts.iter().enumerate() {
		if let std::collections::hash_map::Entry::Vacant(e) = seen.entry(pos_key(p)) {
			e.insert(i);
			distinct.push(i);
		}
	}
	let mut poles: Vec<usize> = Vec::new(); // boundary indices with undefined phi
	let mut ringish: Vec<usize> = Vec::new();
	for &i in &distinct {
		if coords[i].2 {
			ringish.push(i);
		} else {
			if frame.level_cyclic() {
				return Err("a boundary point lies on the torus axis".into());
			}
			poles.push(i);
		}
	}
	if poles.len() > 2 {
		return Err("more than two pole points".into());
	}

	// Group ring candidates into constant-level clusters.
	ringish.sort_by(|&a, &b| coords[a].1.total_cmp(&coords[b].1).then(a.cmp(&b)));
	let mut clusters: Vec<Vec<usize>> = Vec::new();
	for &i in &ringish {
		match clusters.last_mut() {
			Some(c) if (coords[i].1 - coords[*c.last().expect("non-empty cluster")].1).abs() <= RING_ANG_TOL => c.push(i),
			_ => clusters.push(vec![i]),
		}
	}

	// Classify clusters: ≥3 distinct positions must form a uniform full-turn ring;
	// 1–2 positions are seam (slit) points and every index there must be a slit.
	let mut rings: Vec<BoundaryRing> = Vec::new();
	for c in &clusters {
		if c.len() >= 3 {
			let n = c.len();
			let pitch = TAU / n as f64;
			let mut by_phi: Vec<usize> = c.clone();
			by_phi.sort_by(|&a, &b| coords[a].0.total_cmp(&coords[b].0));
			for k in 0..n {
				let gap = wrap_half_turn(coords[by_phi[(k + 1) % n]].0 - coords[by_phi[k]].0).rem_euclid(TAU);
				if (gap - pitch).abs() > RING_ANG_TOL {
					return Err(format!(
						"a boundary circle at level {:.4} is not a uniform full-turn ring (gap {gap:.6} vs pitch {pitch:.6})",
						coords[c[0]].1
					));
				}
			}
			let level = c.iter().map(|&i| coords[i].1).sum::<f64>() / n as f64;
			let slit = c.iter().all(|&i| multiplicity(i) >= 2);
			rings.push(BoundaryRing { level, cols: by_phi, slit });
		} else {
			for &i in c {
				if multiplicity(i) < 2 {
					return Err(format!(
						"boundary point {:?} is neither on a full ring, a pole, nor a seam traversed twice",
						pts[i]
					));
				}
			}
		}
	}

	// All rings must agree on the column count and phase; re-order each ring's
	// samples by column index k (longitude phi0 + k·pitch).
	let n_cols = rings.first().map_or(FULL_TURN_SEGMENTS, |r| r.cols.len());
	if n_cols < 3 {
		return Err("ring with fewer than three samples".into());
	}
	let pitch = TAU / n_cols as f64;
	let phi0 = rings
		.first()
		.map(|r| coords[r.cols[0]].0)
		.unwrap_or_else(|| coords.iter().find(|c| c.2).map(|c| c.0).unwrap_or(0.0));
	for ring in &mut rings {
		if ring.cols.len() != n_cols {
			return Err(format!("rings with mismatched sample counts ({} vs {n_cols})", ring.cols.len()));
		}
		let mut cols = vec![usize::MAX; n_cols];
		for &i in &ring.cols {
			let u = wrap_half_turn(coords[i].0 - phi0).rem_euclid(TAU) / pitch;
			let k = (u.round() as usize) % n_cols;
			if (u - u.round()).abs() * pitch > RING_ANG_TOL || cols[k] != usize::MAX {
				return Err("ring sample longitudes are not aligned with the grid columns".into());
			}
			cols[k] = i;
		}
		ring.cols = cols;
	}
	rings.sort_by(|a, b| a.level.total_cmp(&b.level));

	// Region structure → the ordered row levels (rings, poles, synthesized interior).
	let mut extras: Vec<DVec3> = Vec::new();
	let fresh_ring = |level: f64, extras: &mut Vec<DVec3>| -> GridRow {
		let base = n_pts + extras.len();
		for k in 0..n_cols {
			extras.push(frame.eval(level, phi0 + pitch * k as f64));
		}
		GridRow::Ring((base..base + n_cols).collect())
	};
	// Interior rows between two structural levels, at roughly the ring pitch.
	let interior = |lo: f64, hi: f64| -> Vec<f64> {
		let m = ((hi - lo) / pitch).round().max(1.0) as usize;
		(1..m).map(|k| lo + (hi - lo) * k as f64 / m as f64).collect()
	};

	let mut rows: Vec<GridRow> = Vec::new();
	let mut cyclic = false;
	let pole_level = |i: usize| if coords[i].1 > 0.0 { FRAC_PI_2 } else { -FRAC_PI_2 };
	if frame.level_cyclic() {
		// Torus: 0 rings = a fully periodic cover; 1 slit ring = a full cover anchored
		// at the seam ring; 2 rings = a band spanning the tube angle between them.
		cyclic = rings.len() < 2;
		match rings.len() {
			0 => {
				let anchor = coords.iter().find(|c| c.2).map(|c| c.1).unwrap_or(0.0);
				for k in 0..n_cols {
					let lv = anchor + TAU * k as f64 / n_cols as f64;
					rows.push(fresh_ring(lv, &mut extras));
				}
			}
			1 => {
				let r0 = rings.remove(0);
				if !r0.slit {
					return Err("a single non-slit ring cannot bound a torus region".into());
				}
				let lv0 = r0.level;
				rows.push(GridRow::Ring(r0.cols));
				let m = (TAU / pitch).round() as usize;
				for k in 1..m {
					rows.push(fresh_ring(lv0 + TAU * k as f64 / m as f64, &mut extras));
				}
			}
			2 => {
				let hi = rings.pop().expect("two rings");
				let lo = rings.pop().expect("two rings");
				// The tube angle wraps, so "between the rims" is ambiguous: the loop's
				// region side at each rim decides which of the two bands the face is.
				let side_lo = ring_region_side(pts, &lo, frame, same_sense)?;
				let side_hi = ring_region_side(pts, &hi, frame, same_sense)?;
				if side_lo == side_hi {
					return Err("the two torus rims claim the same region side".into());
				}
				let (start, end) = if side_lo > 0.0 { (lo, hi) } else { (hi, lo) };
				let span = (end.level - start.level).rem_euclid(TAU);
				let m = (span / pitch).round().max(1.0) as usize;
				let start_level = start.level;
				rows.push(GridRow::Ring(start.cols));
				for k in 1..m {
					rows.push(fresh_ring(start_level + span * k as f64 / m as f64, &mut extras));
				}
				rows.push(GridRow::Ring(end.cols));
			}
			n => return Err(format!("{n} rings on a torus face (only a band between two rims is importable)")),
		}
	} else {
		// Sphere: cap (1 ring [+ pole]), band (2 rings), or full sphere (poles only).
		match (rings.len(), poles.len()) {
			(0, 2) => {
				let (s, n) = if pole_level(poles[0]) < 0.0 { (poles[0], poles[1]) } else { (poles[1], poles[0]) };
				if pole_level(s) >= 0.0 || pole_level(n) <= 0.0 {
					return Err("two pole points on the same side of the sphere".into());
				}
				rows.push(GridRow::Pole(s));
				for lv in interior(-FRAC_PI_2, FRAC_PI_2) {
					rows.push(fresh_ring(lv, &mut extras));
				}
				rows.push(GridRow::Pole(n));
			}
			(1, np @ 0..=1) => {
				let ring = rings.remove(0);
				// Region side: toward the boundary pole if present, else the side the
				// loop encircles (its circulation about the axis, oriented by the
				// material normal `same_sense ? outward : inward`).
				let north = if np == 1 {
					pole_level(poles[0]) > 0.0
				} else {
					let side = ring_region_side(pts, &ring, frame, same_sense)?;
					side > 0.0
				};
				let pole_row = if np == 1 {
					GridRow::Pole(poles[0])
				} else {
					extras.push(frame.pole(north));
					GridRow::Pole(n_pts + extras.len() - 1)
				};
				let target = if north { FRAC_PI_2 } else { -FRAC_PI_2 };
				let inner = interior(ring.level.min(target), ring.level.max(target));
				if north {
					rows.push(GridRow::Ring(ring.cols));
					for lv in inner {
						rows.push(fresh_ring(lv, &mut extras));
					}
					rows.push(pole_row);
				} else {
					rows.push(pole_row);
					for lv in inner {
						rows.push(fresh_ring(lv, &mut extras));
					}
					rows.push(GridRow::Ring(ring.cols));
				}
			}
			(2, 0) => {
				let hi = rings.pop().expect("two rings");
				let lo = rings.pop().expect("two rings");
				// The band between the rims is the only candidate region on a sphere
				// (its complement is disconnected) — but the rims must agree.
				let side_lo = ring_region_side(pts, &lo, frame, same_sense)?;
				let side_hi = ring_region_side(pts, &hi, frame, same_sense)?;
				if !(side_lo > 0.0 && side_hi < 0.0) {
					return Err("the sphere band rims do not face each other".into());
				}
				rows.push(GridRow::Ring(lo.cols));
				for lv in interior(lo.level, hi.level) {
					rows.push(fresh_ring(lv, &mut extras));
				}
				rows.push(GridRow::Ring(hi.cols));
			}
			(nr, np) => {
				return Err(format!(
					"{nr} ring(s) + {np} pole(s) is not a sphere cap, band or full sphere"
				))
			}
		}
	}
	if rows.len() < 2 {
		return Err("the region resolves to fewer than two grid rows".into());
	}

	// Every boundary point must now be consumed: a ring/pole member, or a slit point
	// (its two traversals intern to the same vertex and pair internally).
	let mut used = vec![false; n_pts];
	for row in &rows {
		match row {
			GridRow::Ring(cols) => {
				for &h in cols {
					if h < n_pts {
						used[h] = true;
					}
				}
			}
			GridRow::Pole(h) => {
				if *h < n_pts {
					used[*h] = true;
				}
			}
		}
	}
	// Mark every index sharing a used position, then require leftovers to be slits.
	let used_keys: std::collections::HashSet<PosKey> =
		(0..n_pts).filter(|&i| used[i]).map(|i| pos_key(pts[i])).collect();
	for (i, &p) in pts.iter().enumerate() {
		if !used_keys.contains(&pos_key(p)) && multiplicity(i) < 2 {
			return Err(format!("boundary point {p:?} was not consumed by the ring grid"));
		}
	}

	// Emit the facet quads/fans between consecutive rows.
	let mut tris: Vec<[usize; 3]> = Vec::new();
	let row_pairs = rows.len() - 1 + usize::from(cyclic);
	for r in 0..row_pairs {
		let a = &rows[r % rows.len()];
		let b = &rows[(r + 1) % rows.len()];
		match (a, b) {
			(GridRow::Ring(ra), GridRow::Ring(rb)) => {
				for k in 0..n_cols {
					let k1 = (k + 1) % n_cols;
					tris.push([ra[k], ra[k1], rb[k1]]);
					tris.push([ra[k], rb[k1], rb[k]]);
				}
			}
			(GridRow::Ring(ra), GridRow::Pole(p)) => {
				for k in 0..n_cols {
					tris.push([ra[k], ra[(k + 1) % n_cols], *p]);
				}
			}
			(GridRow::Pole(p), GridRow::Ring(rb)) => {
				for k in 0..n_cols {
					tris.push([*p, rb[(k + 1) % n_cols], rb[k]]);
				}
			}
			(GridRow::Pole(_), GridRow::Pole(_)) => return Err("two adjacent pole rows".into()),
		}
	}

	// Orientation: the facets must traverse a (non-slit) boundary ring exactly as the
	// loop does — that is what pairs them with the neighbour face's edges. A fully
	// periodic face has only slit boundaries (no net loop winding), so its global
	// orientation comes from `same_sense` against the analytic outward normal.
	let flip = if let Some((i, j)) = loop_ring_step(pts, &rows, n_pts, n_cols) {
		// The loop steps i→j along a ring; the canonical facets step ra[k]→ra[k+1].
		// Find their column indices and compare directions.
		let row = rows
			.iter()
			.find_map(|r| match r {
				GridRow::Ring(cols) if cols.contains(&i) && cols.contains(&j) => Some(cols),
				_ => None,
			})
			.expect("loop_ring_step returned members of one ring row");
		let ki = row.iter().position(|&h| h == i).expect("i in row");
		let kj = row.iter().position(|&h| h == j).expect("j in row");
		kj != (ki + 1) % n_cols
	} else {
		// Slits only: compare one facet's winding against the surface normal.
		let handle = |h: usize| if h < n_pts { pts[h] } else { extras[h - n_pts] };
		let t = tris.first().expect("at least one facet");
		let (a, b, c) = (handle(t[0]), handle(t[1]), handle(t[2]));
		let fn_ = (b - a).cross(c - a);
		let centroid = (a + b + c) / 3.0;
		let (phi, level, _) = frame.coords(centroid);
		let outward = frame.normal(level, phi);
		(fn_.dot(outward) > 0.0) != same_sense
	};
	if flip {
		for t in &mut tris {
			t.swap(1, 2);
		}
	}
	Ok((extras, tris))
}

/// Which side of a boundary ring the face region lies on: `+1` toward increasing
/// level, `-1` toward decreasing — from the loop's traversal direction `d` at a ring
/// sample, the material normal `n` (`same_sense` selects outward/inward) and the
/// level direction `t`: the region is to the LEFT of the walk, `sign((n × d) · t)`.
fn ring_region_side(pts: &[DVec3], ring: &BoundaryRing, frame: &PeriodicFrame, same_sense: bool) -> Result<f64, String> {
	if ring.slit {
		return Err("cannot take a region side from a slit ring".into());
	}
	let n_pts = pts.len();
	let member: std::collections::HashSet<usize> = ring.cols.iter().copied().collect();
	for i in 0..n_pts {
		let j = (i + 1) % n_pts;
		if member.contains(&i) && member.contains(&j) {
			let d = pts[j] - pts[i];
			let (phi, level, _) = frame.coords(pts[i]);
			let n = frame.normal(level, phi) * if same_sense { 1.0 } else { -1.0 };
			let s = n.cross(d).dot(frame.level_dir(level, phi));
			if s.abs() > 1e-12 {
				return Ok(s.signum());
			}
		}
	}
	Err("no loop step along the rim ring to take a region side from".into())
}

/// The first loop step `pts[i] → pts[i+1]` whose endpoints are distinct members of
/// one NON-slit ring row — the orientation witness for the facet winding. `None`
/// when every ring is a slit (fully periodic faces).
fn loop_ring_step(pts: &[DVec3], rows: &[GridRow], n_pts: usize, n_cols: usize) -> Option<(usize, usize)> {
	for row in rows {
		let GridRow::Ring(cols) = row else { continue };
		// Boundary ring rows hold input indices; synthesized rows hold extras.
		if cols.iter().any(|&h| h >= n_pts) {
			continue;
		}
		// A slit ring is traversed both ways; its steps would be ambiguous. Detect by
		// occurrence: if any directed step appears in BOTH directions, skip the ring.
		let member: std::collections::HashSet<usize> = cols.iter().copied().collect();
		let mut steps: Vec<(usize, usize)> = Vec::new();
		for i in 0..n_pts {
			let j = (i + 1) % n_pts;
			if member.contains(&i) && member.contains(&j) && i != j {
				steps.push((i, j));
			}
		}
		let keyed: std::collections::HashSet<(PosKey, PosKey)> =
			steps.iter().map(|&(i, j)| (pos_key(pts[i]), pos_key(pts[j]))).collect();
		let two_way = steps.iter().any(|&(i, j)| keyed.contains(&(pos_key(pts[j]), pos_key(pts[i]))));
		if two_way {
			continue;
		}
		if let Some(&(i, j)) = steps.first() {
			let _ = n_cols;
			return Some((i, j));
		}
	}
	None
}

/// Samples per direction of the coarse seed grid used to initialise Newton
/// projection of trim-loop vertices onto a B-spline patch.
pub(crate) const PATCH_SEED_GRID: usize = 24;

/// Relative distance (scaled by `1 + |p|`) within which a trim-loop vertex must land
/// on its B-spline patch after Newton projection. Real exporters keep trim curves
/// within ~1e-7 of the surface; anything farther means the loop does not actually
/// bound a region of this patch and the face is refused loudly. (Shared with the
/// exporter's patch-coverage test in [`crate::step_export`].)
pub(crate) const PATCH_PROJECT_TOL: f64 = 1e-6;

/// Relative chordal tolerance of a trimmed B-spline face's interior facets: an
/// interior chord is bisected while it deviates from the exact patch by more than
/// this fraction of the face's own scale (`1 + max |trim vertex|`). At `1e-3` the
/// facets match the imported-conic fidelity contract — a 48-segment ring's chord
/// sagitta is `2.1e-3·r` — while rulings (zero deviation at any length) are left
/// whole. (Shared with the exporter's patch-coverage test in
/// [`crate::step_export`]: a facet of the patch's own tessellation sits within
/// this sag of the patch, which is far looser than the trim-vertex projection
/// tolerance.)
pub(crate) const PATCH_SAG_TOL: f64 = 1e-3;

/// Hard cap on facets per trimmed B-spline face (a refusal beats an unbounded blowup
/// on a pathological patch).
const PATCH_FACET_BUDGET: usize = 20_000;

/// Minimum interior facet pitch, in NORMALISED parameter space: refinement never
/// splits an edge all of whose owner triangles have (twice-)area at or below
/// `PATCH_MIN_PITCH²/2` — the **area floor** (the W3 termination device, kept under
/// the chordal criterion). It is what stops the sliver cascade against an
/// unsplittable long trim chord: the boundary is pinned by the weld, so the strip
/// hugging it can only be "refined" by driving interior vertices asymptotically
/// onto the chord — infinitely many splits, hair-width facets that break the
/// downstream weld. With the floor the strip pins at ~the floor width and its
/// residual chordal error stays bounded (≲ sag(strip) · strip width — far inside
/// the volume fidelity budget); everywhere else 1/256 is far finer than the
/// [`PATCH_FACET_BUDGET`] could fill anyway, so the floor never bites real
/// curvature refinement.
const PATCH_MIN_PITCH: f64 = 1.0 / 256.0;

/// Twice-the-area floor derived from [`PATCH_MIN_PITCH`] (`pitch²/4` of true area).
const AREA_FLOOR2: f64 = PATCH_MIN_PITCH * PATCH_MIN_PITCH / 2.0;

/// The coarse `(normalised uv, position)` seed grid of a patch, evaluated once per
/// face and shared across all of its trim-vertex projections
/// ([`NurbsSurface::projection_seeds`] at the [`PATCH_SEED_GRID`] resolution).
fn patch_seed_grid(surf: &NurbsSurface) -> Vec<(DVec2, DVec3)> {
	surf.projection_seeds(PATCH_SEED_GRID)
}

/// Invert the patch at `p`: the normalised `(u, v) ∈ [0,1]²` whose surface point is
/// `p` ([`NurbsSurface::project`]). `None` when the converged point is farther than
/// [`PATCH_PROJECT_TOL`] — i.e. `p` is not on the patch.
fn uv_on_patch(surf: &NurbsSurface, grid: &[(DVec2, DVec3)], p: DVec3) -> Option<DVec2> {
	surf.project(grid, p, PATCH_PROJECT_TOL)
}

/// Undirected edge key.
fn edge_key(a: usize, b: usize) -> (usize, usize) {
	(a.min(b), a.max(b))
}

/// Whether a patch is geometrically closed (periodic) across its `u` domain ends —
/// `S(u_lo, v) = S(u_hi, v)` along the whole seam to relative tolerance. These are
/// the closed patches (NURBS cylinders and friends) whose seam a trim loop may
/// legitimately cross, handled by unwrapping into the universal cover. `in_u =
/// false` checks the `v` direction. The check EVALUATES the surface (not a
/// control-net heuristic), so a wrapped net with mismatched weight rows — whose
/// seam genuinely gapes — does not false-positive.
fn patch_closed(surf: &NurbsSurface, in_u: bool) -> bool {
	let ((u_lo, u_hi), (v_lo, v_hi)) = surf.domain();
	(0..=4).all(|k| {
		let f = k as f64 / 4.0;
		let (a, b) = if in_u {
			let v = v_lo + (v_hi - v_lo) * f;
			(surf.point_at(u_lo, v), surf.point_at(u_hi, v))
		} else {
			let u = u_lo + (u_hi - u_lo) * f;
			(surf.point_at(u, v_lo), surf.point_at(u, v_hi))
		};
		(a - b).length() <= 1e-9 * (1.0 + a.length().max(b.length()))
	})
}

/// Unwrap one trim ring's normalised `uv` into the universal cover of a closed
/// patch: every step is taken the short way around (within half a period — the same
/// half-turn convention as the analytic periodic wall), so a chord crossing the
/// parameter seam continues into the neighbouring period instead of jumping back
/// across the domain, and a seam edge's two traversals land one period apart (the
/// duplicated seam parameters; their identical 3-D positions weld on interning).
/// Returns the loop's net winding in whole periods per direction — `0` for every
/// disk-bounding loop. The closing chord back to the first vertex is also a
/// short-way step, so the winding is `round(last − first)` exactly.
fn unwrap_ring(uv: &mut [DVec2], ring: &[usize], closed_u: bool, closed_v: bool) -> (i64, i64) {
	let wrap = |d: f64| d - d.round();
	for k in 1..ring.len() {
		let (prev, cur) = (uv[ring[k - 1]], uv[ring[k]]);
		if closed_u {
			uv[ring[k]].x = prev.x + wrap(cur.x - prev.x);
		}
		if closed_v {
			uv[ring[k]].y = prev.y + wrap(cur.y - prev.y);
		}
	}
	let (first, last) = (uv[ring[0]], uv[ring[ring.len() - 1]]);
	let winding = |closed: bool, f: f64, l: f64| if closed { (l - f).round() as i64 } else { 0 };
	(winding(closed_u, first.x, last.x), winding(closed_v, first.y, last.y))
}

/// Bridge the two full-period rim rings of an *untrimmed* closed patch (a band
/// covering one whole closed direction, e.g. a NURBS tube wall bounded only by its
/// two rims) into ONE disk-bounding ring in the universal cover:
///
/// - rim `a` (winding `+1`/`−1` along the closed direction `in_u`) is extended with a
///   duplicate of its first vertex one period along its travel, so the chain spans
///   exactly one period and both ends carry the same 3-D position;
/// - rim `b` (the opposite winding) is rotated to start nearest the extended end of
///   `a`, shifted by whole periods to land there, and extended the same way;
/// - the two chains are concatenated. The two connecting chords (`a`-end → `b`-start
///   and `b`-end → `a`-start) are one period apart in the cover but bit-identical in
///   3-D — a synthetic seam whose two copies intern to the same vertices and pair as
///   twins, exactly like a real exporter's `SEAM_CURVE` slit.
///
/// New cover vertices (the two duplicates) are appended to `uv`/`pts3`. The merged
/// ring keeps both rims' input traversal directions, so every rim chord still pairs
/// with the neighbouring cap's edges.
fn bridge_band_rings(
	uv: &mut Vec<DVec2>,
	pts3: &mut Vec<DVec3>,
	ring_a: &[usize],
	ring_b: &[usize],
	in_u: bool,
) -> Vec<usize> {
	let coord = |q: DVec2| if in_u { q.x } else { q.y };
	let with_coord = |q: DVec2, c: f64| if in_u { DVec2::new(c, q.y) } else { DVec2::new(q.x, c) };
	// `dir` = the sign of rim a's winding (its chain ascends or descends one period).
	let dir = (coord(uv[*ring_a.last().expect("rims are non-empty")]) - coord(uv[ring_a[0]])).signum();
	// a-chain closure: duplicate a's first vertex one period along its travel.
	let a_dup = uv.len();
	uv.push(with_coord(uv[ring_a[0]], coord(uv[ring_a[0]]) + dir));
	pts3.push(pts3[ring_a[0]]);
	let target = coord(uv[a_dup]);
	// Rotate b to the entry whose period-shifted coordinate lands nearest `target`.
	let nearest = |i: usize| {
		let c = coord(uv[ring_b[i]]);
		(c - target) - (c - target).round()
	};
	let n_b = ring_b.len();
	let rot = (0..n_b)
		.min_by(|&i, &j| nearest(i).abs().total_cmp(&nearest(j).abs()))
		.expect("rims are non-empty");
	let shift = (target - coord(uv[ring_b[rot]])).round();
	// Re-anchor b's cover coordinates: rotated order, continuing b's own winding
	// (−dir) across its original wrap point, then the whole-period shift.
	let mut merged: Vec<usize> = ring_a.to_vec();
	merged.push(a_dup);
	for k in 0..n_b {
		let idx = ring_b[(rot + k) % n_b];
		let mut c = coord(uv[idx]) + shift;
		if rot + k >= n_b {
			c -= dir; // b winds opposite a: its wrapped-around prefix continues one period further
		}
		uv[idx] = with_coord(uv[idx], c);
		merged.push(idx);
	}
	// b-chain closure: duplicate b's (rotated) first vertex one period along ITS travel.
	let b_dup = uv.len();
	uv.push(with_coord(uv[ring_b[rot]], coord(uv[ring_b[rot]]) - dir));
	pts3.push(pts3[ring_b[rot]]);
	merged.push(b_dup);
	merged
}

/// Ear-clip the trimming region of a parameter-space polygon with holes into index
/// triangles **wound like the outer input ring**. `rings[0]` is the outer loop,
/// `rings[1..]` the holes, each a ring of indices into `uv` in loop order. Holes are
/// bridged into the outer ring through their max-`x` vertex and the nearest visible
/// outer vertex (a doubled zero-width edge), then the merged simple polygon is
/// clipped with exact orientation tests — the same construction the planar
/// tessellator uses, but index-returning so boundary handles survive for the
/// watertight weld. Degenerate or self-crossing trim loops error with a reason.
fn triangulate_trim_rings(uv: &[DVec2], rings: &[Vec<usize>]) -> Result<Vec<[usize; 3]>, String> {
	let signed_area = |ring: &[usize]| -> f64 {
		let n = ring.len();
		(0..n)
			.map(|i| {
				let a = uv[ring[i]];
				let b = uv[ring[(i + 1) % n]];
				a.x * b.y - b.x * a.y
			})
			.sum::<f64>()
			* 0.5
	};
	let outer_area = signed_area(&rings[0]);
	if outer_area == 0.0 {
		return Err("the outer trimming loop encloses no parameter-space area".into());
	}
	// The clipper works in ABSOLUTE parameter orientation: outer CCW, holes CW.
	// `flipped` records whether the input outer was CW, so the emitted triangle
	// windings can be swapped back to match the input at the end.
	let flipped = outer_area < 0.0;
	let orient_ring = |ring: &[usize], ccw: bool| -> Vec<usize> {
		let mut r = ring.to_vec();
		if (signed_area(ring) > 0.0) != ccw {
			r.reverse();
		}
		r
	};
	let mut outer = orient_ring(&rings[0], true);
	let mut holes: Vec<Vec<usize>> = rings[1..].iter().map(|h| orient_ring(h, false)).collect();
	// Bridge right-most holes first so later bridges cannot cross them.
	holes.sort_by(|a, b| {
		let mx = |r: &[usize]| r.iter().map(|&i| uv[i].x).fold(f64::NEG_INFINITY, f64::max);
		mx(b).total_cmp(&mx(a))
	});
	let proper_cross = |a: DVec2, b: DVec2, c: DVec2, d: DVec2| -> bool {
		let o = |p: DVec2, q: DVec2, r: DVec2| orient2d([p.x, p.y], [q.x, q.y], [r.x, r.y]);
		o(c, d, a) * o(c, d, b) < 0.0 && o(a, b, c) * o(a, b, d) < 0.0
	};
	let all_holes = holes.clone();
	for hole in &holes {
		// The hole's right-most vertex sees outward; bridge it to the nearest outer
		// vertex with an uncrossed segment.
		let &h = hole
			.iter()
			.max_by(|&&i, &&j| uv[i].x.total_cmp(&uv[j].x))
			.expect("holes are non-empty rings");
		let mut candidates: Vec<usize> = (0..outer.len()).collect();
		candidates.sort_by(|&i, &j| (uv[outer[i]] - uv[h]).length_squared().total_cmp(&(uv[outer[j]] - uv[h]).length_squared()));
		let visible = |o_idx: usize| -> bool {
			let (pa, pb) = (uv[outer[o_idx]], uv[h]);
			let clear = |ring: &[usize]| {
				let n = ring.len();
				(0..n).all(|i| {
					let (c, d) = (ring[i], ring[(i + 1) % n]);
					c == outer[o_idx] || c == h || d == outer[o_idx] || d == h || !proper_cross(pa, pb, uv[c], uv[d])
				})
			};
			clear(&outer) && all_holes.iter().all(|r| clear(r))
		};
		let Some(&o_idx) = candidates.iter().find(|&&i| visible(i)) else {
			return Err("a trimming hole has no uncrossed bridge to the outer loop".into());
		};
		// Splice: …outer[o], hole[h], hole around, hole[h], outer[o]… (doubled bridge).
		let h_pos = hole.iter().position(|&i| i == h).expect("h came from this hole");
		let mut merged = Vec::with_capacity(outer.len() + hole.len() + 2);
		merged.extend_from_slice(&outer[..=o_idx]);
		merged.extend(hole[h_pos..].iter().copied());
		merged.extend(hole[..=h_pos].iter().copied());
		merged.extend_from_slice(&outer[o_idx..]);
		outer = merged;
	}
	// Ear clipping with exact orientation; coincident copies of a corner (the bridge
	// twins) never block an ear — they intern to the same 3-D vertex anyway.
	let mut idx = outer;
	let mut tris: Vec<[usize; 3]> = Vec::with_capacity(idx.len().saturating_sub(2));
	while idx.len() > 3 {
		let n = idx.len();
		let mut clipped = false;
		for i in 0..n {
			let (ip, ic, inx) = (idx[(i + n - 1) % n], idx[i], idx[(i + 1) % n]);
			let (a, b, c) = (uv[ip], uv[ic], uv[inx]);
			if orient2d([a.x, a.y], [b.x, b.y], [c.x, c.y]) <= 0.0 {
				continue; // reflex or flat corner
			}
			let blocked = idx.iter().any(|&j| {
				if j == ip || j == ic || j == inx {
					return false;
				}
				let p = uv[j];
				if p == a || p == b || p == c {
					return false; // a bridge twin of one of the corners
				}
				let sign = |p1: DVec2, p2: DVec2, p3: DVec2| orient2d([p3.x, p3.y], [p1.x, p1.y], [p2.x, p2.y]);
				let (d1, d2, d3) = (sign(p, a, b), sign(p, b, c), sign(p, c, a));
				let has_neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
				let has_pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
				!(has_neg && has_pos)
			});
			if !blocked {
				tris.push([ip, ic, inx]);
				idx.remove(i);
				clipped = true;
				break;
			}
		}
		if !clipped {
			return Err("the trimming loops do not bound a simple parameter-space region".into());
		}
	}
	if idx.len() == 3 {
		let (a, b, c) = (uv[idx[0]], uv[idx[1]], uv[idx[2]]);
		if orient2d([a.x, a.y], [b.x, b.y], [c.x, c.y]) > 0.0 {
			tris.push([idx[0], idx[1], idx[2]]);
		} else if !(a == b || b == c || c == a) {
			return Err("the final trimming ear is inverted".into());
		}
	}
	if flipped {
		for t in &mut tris {
			t.swap(1, 2);
		}
	}
	Ok(tris)
}

/// Conforming **chordal** refinement of a parameter-space triangulation. An interior
/// edge *qualifies* while its straight 3-D chord deviates from the exact surface by
/// more than `sag_tol` (the *sagitta* `|S(uv mid) − (P(a)+P(b))/2|`); each round
/// takes the worst qualifying edge and bisects — not that edge directly, but the
/// **terminal longest edge** reached by walking Rivara's longest-edge chain from it
/// (while an adjacent triangle has a strictly longer non-boundary edge, move to it).
/// Longest-edge bisection keeps the aspect ratio of every triangle bounded, which is
/// what prevents sliver cascades: bisecting an arbitrary qualifying (often short,
/// curvature-spanning) edge breeds ever-thinner slivers whose crossing midpoints pile
/// up within 1e-11 of each other and collapse into degenerate facets downstream.
/// Both owner triangles split at one shared midpoint, so the mesh never grows a
/// T-junction. Edges in `boundary` (the trim-loop segments — the weld with
/// neighbouring faces) are never split, so the loop chords stay exactly the
/// producer's. `pos` runs parallel to `uv` (the exact 3-D position of every handle:
/// boundary verbatim, interior evaluated); each new midpoint is appended to both,
/// evaluated through `eval` (which wraps the cover coordinates of a closed patch
/// back into the domain).
///
/// Chord deviation — not parameter length — is the honest qualification: a chord
/// along a ruling of the surface (a closed tube's seam, a flat patch's diagonal) is
/// geometrically exact at ANY parameter length and is left alone, while an
/// arc-spanning chord is refined until faithful. Termination is NOT left to the
/// sagitta's quadratic decay alone — against an unsplittable trim chord that decay
/// stalls (the boundary strip can only thin asymptotically) and interned-midpoint
/// splits can even cycle with zero net progress — but is enforced by three layers:
/// the [`AREA_FLOOR2`] qualification (the W3 device: split areas strictly descend,
/// sub-floor strips are pinned and their bounded residual sag accepted), the
/// live-owner walk filter (the walk never enters the sub-floor web), and a loud
/// round cap behind the [`PATCH_FACET_BUDGET`] (a refusal beats a hang on a
/// pathological patch).
fn refine_param_facets(
	uv: &mut Vec<DVec2>,
	pos: &mut Vec<DVec3>,
	tris: &mut Vec<[usize; 3]>,
	boundary: &std::collections::HashSet<(usize, usize)>,
	eval: impl Fn(DVec2) -> DVec3,
	sag_tol: f64,
) -> Result<(), String> {
	// Midpoints are interned by exact uv bits: on symmetric patches different splits
	// can land on the SAME parameter point, and giving each landing a fresh index
	// would let geometrically identical edges coexist under distinct index pairs —
	// each split blind to the others, regenerating one another forever. Interning
	// makes such a coincidence a shared vertex instead; a split whose midpoint IS an
	// owner's third vertex then simply retires that owner (it was a zero-area sliver
	// astride the vertex), keeping the triangulation conforming.
	let uv_key = |q: DVec2| (q.x.to_bits(), q.y.to_bits());
	let mut by_uv: HashMap<(u64, u64), usize> = uv.iter().enumerate().map(|(i, &q)| (uv_key(q), i)).collect();
	// An edge's sagitta is immutable (its endpoints' uv/pos never change), so it is
	// evaluated once per index pair, not once per pass.
	let mut sag_cache: HashMap<(usize, usize), f64> = HashMap::new();
	// Safety net behind the live-owner walk filter below: a split that lands on an
	// interned midpoint can leave the facet count unchanged, so the facet budget
	// alone does not bound the LOOP — cap the rounds outright and refuse loudly
	// (a refusal beats a silent hang on a pathological patch).
	let mut rounds = 0usize;
	loop {
		if tris.len() > PATCH_FACET_BUDGET {
			return Err(format!("patch refinement exceeded the {PATCH_FACET_BUDGET}-facet budget"));
		}
		rounds += 1;
		if rounds > 8 * PATCH_FACET_BUDGET {
			return Err(format!(
				"patch refinement failed to converge within {} rounds (degenerate-sliver cycling)",
				8 * PATCH_FACET_BUDGET
			));
		}
		let mut adj: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
		for (ti, t) in tris.iter().enumerate() {
			for k in 0..3 {
				adj.entry(edge_key(t[k], t[(k + 1) % 3])).or_default().push(ti);
			}
		}
		// The qualifying edge with the worst chord deviation. Edges all of whose
		// owners sit at or below the area floor are excluded — that is both the
		// termination device (split areas strictly descend to the floor) and the
		// guard against the boundary-strip sliver cascade (see [`PATCH_MIN_PITCH`]).
		let area2 = |t: &[usize; 3]| (uv[t[1]] - uv[t[0]]).perp_dot(uv[t[2]] - uv[t[0]]).abs();
		let target = adj
			.iter()
			.map(|(&(a, b), owners)| {
				let s = if boundary.contains(&(a, b)) || !owners.iter().any(|&ti| area2(&tris[ti]) > AREA_FLOOR2) {
					0.0
				} else {
					*sag_cache
						.entry((a, b))
						.or_insert_with(|| (eval((uv[a] + uv[b]) * 0.5) - (pos[a] + pos[b]) * 0.5).length())
				};
				(a, b, s)
			})
			.filter(|&(_, _, s)| s > sag_tol)
			// Ties break on the index pair so the split order is deterministic.
			.max_by(|x, y| x.2.total_cmp(&y.2).then_with(|| (x.0, x.1).cmp(&(y.0, y.1))));
		let Some((qa, qb, _)) = target else {
			return Ok(());
		};
		// Rivara walk: while an owner of the current edge has a strictly longer
		// non-boundary edge WITH a live (above-floor) owner of its own, move to
		// (the longest) one. Lengths strictly increase, so the walk is finite; the
		// cap is pure paranoia. The live-owner condition mirrors the target filter
		// and is load-bearing: sub-floor residue can include a web of EXACTLY
		// degenerate slivers (collinear vertices along a trim chord) whose edges,
		// split at midpoints interned to EXISTING vertices, recreate one another
		// two-cyclically — zero net progress, an infinite loop the facet budget
		// never catches because the count never grows. An edge with a live owner
		// always splits that owner into two genuine halves.
		let len2 = |x: usize, y: usize| (uv[x] - uv[y]).length_squared();
		let live = |key: (usize, usize), adj: &HashMap<(usize, usize), Vec<usize>>, tris: &[[usize; 3]]| {
			adj.get(&key).is_some_and(|own| own.iter().any(|&ti| area2(&tris[ti]) > AREA_FLOOR2))
		};
		let (mut a, mut b) = (qa, qb);
		for _ in 0..3 * tris.len() + 8 {
			let owners = adj.get(&edge_key(a, b)).ok_or("the refinement walk left the triangulation")?;
			let mut next: Option<((usize, usize), f64)> = None;
			for &ti in owners {
				let t = tris[ti];
				for k in 0..3 {
					let ek = edge_key(t[k], t[(k + 1) % 3]);
					let l = len2(ek.0, ek.1);
					if !boundary.contains(&ek)
						&& l > len2(a, b)
						&& live(ek, &adj, tris)
						&& next.is_none_or(|(bk, bl)| l.total_cmp(&bl).then_with(|| ek.cmp(&bk)) == std::cmp::Ordering::Greater)
					{
						next = Some((ek, l));
					}
				}
			}
			match next {
				Some(((x, y), _)) => (a, b) = (x, y),
				None => break,
			}
		}
		let owners = adj.get(&edge_key(a, b)).expect("the walk ends on a live edge");
		if owners.len() > 2 {
			return Err("a parameter-space edge borders more than two facets".into());
		}
		let mid = (uv[a] + uv[b]) * 0.5;
		let m = *by_uv.entry(uv_key(mid)).or_insert_with(|| {
			uv.push(mid);
			pos.push(eval(mid));
			uv.len() - 1
		});
		// Split each owner along the a–b edge, preserving its winding. An owner whose
		// third vertex IS the (interned) midpoint is a zero-area sliver lying along
		// the edge: both its children would be degenerate, so it is dropped — its two
		// half-edges remain covered by the neighbours' children.
		let mut owner_idx = owners.clone();
		owner_idx.sort_unstable_by(|x, y| y.cmp(x)); // remove from the back first
		for ti in owner_idx {
			let t = tris.swap_remove(ti);
			let r = *t.iter().find(|&&v| v != a && v != b).expect("a triangle has a third vertex");
			if r == m {
				continue;
			}
			// The directed a→b (or b→a) occurrence fixes the two children's winding.
			let forward = (0..3).any(|k| t[k] == a && t[(k + 1) % 3] == b);
			if forward {
				tris.push([a, m, r]);
				tris.push([m, b, r]);
			} else {
				tris.push([b, m, r]);
				tris.push([m, a, r]);
			}
		}
	}
}

/// Import one trimmed `B_SPLINE_SURFACE_WITH_KNOTS` face by tessellating it **on the
/// exact patch**: every trim-loop vertex is Newton-projected into the patch's
/// parameter space ([`uv_on_patch`] — a vertex off the surface is a loud refusal);
/// on a CLOSED (periodic) patch the loops are unwrapped into the universal cover
/// ([`unwrap_ring`], [`bridge_band_rings`]) so seam-crossing/slit loops and
/// two-rim bands import instead of refusing; the loops are then triangulated in
/// parameter space (monotone sweep for a single ring, hole-bridging ear clip via
/// [`triangulate_trim_rings`] otherwise) and the interior is refined to the
/// [`PATCH_SAG_TOL`] relative chordal tolerance with every new vertex EVALUATED on
/// the exact surface via [`NurbsSurface::point_at`]. Trim-loop chords are never
/// subdivided, so the boundary stays bit-identical with the neighbouring faces'
/// edges and the weld is watertight.
///
/// Tagging design (least-invasive, by ownership): the analytic [`Surface`] enum has
/// no freeform variant, so each emitted chord facet carries its own exact
/// `Surface::Plane` tag (a triangle IS its plane — the tag is geometrically true).
/// The patch's NURBS identity is therefore not carried on the [`Solid`] itself; it
/// IS preserved in the [`FreeformFace`] sidecar ([`import_step_freeform`]), and
/// exact patch reads stay available via [`import_bspline_surface`].
fn add_bspline_face(
	imp: &Importer,
	fid: u32,
	surface_ref: u32,
	outer_pts: &[DVec3],
	inner_loops: &[Vec<DVec3>],
	acc: &mut FaceAccum,
) -> Result<(), StepError> {
	let surf = imp.bspline_surface(surface_ref)?;
	let ((u_lo, u_hi), (v_lo, v_hi)) = surf.domain();
	if !(u_hi > u_lo && v_lo < v_hi) {
		return Err(StepError::Unsupported(format!(
			"ADVANCED_FACE #{fid}: B-spline patch #{surface_ref} has a degenerate parameter domain"
		)));
	}
	let grid = patch_seed_grid(&surf);
	// Project every trim-loop vertex into normalised parameter space.
	let mut pts3: Vec<DVec3> = Vec::new();
	let mut uv: Vec<DVec2> = Vec::new();
	let mut rings: Vec<Vec<usize>> = Vec::new();
	for lp in std::iter::once(outer_pts).chain(inner_loops.iter().map(Vec::as_slice)) {
		let base = pts3.len();
		for &p in lp {
			let Some(q) = uv_on_patch(&surf, &grid, p) else {
				return Err(StepError::Unsupported(format!(
					"ADVANCED_FACE #{fid}: trim vertex ({:.4}, {:.4}, {:.4}) does not lie on B-spline patch #{surface_ref}",
					p.x, p.y, p.z
				)));
			};
			pts3.push(p);
			uv.push(q);
		}
		rings.push((base..pts3.len()).collect());
	}
	// Closed (periodic) patch directions: a trim loop may legitimately cross the
	// parameter seam (a pocket milled across it) or traverse it as a doubled slit
	// (a real exporter's closed tube wall). Unwrap every ring into the universal
	// cover ([`unwrap_ring`]): seam-crossing chords continue into the neighbouring
	// period and a seam edge's two traversals land one period apart, welding back
	// in 3-D on interning. Each per-direction step is taken the SHORT way around —
	// a single trim chord deliberately spanning more than half a period of a closed
	// direction is indistinguishable from a seam crossing and is read as one.
	let (closed_u, closed_v) = (patch_closed(&surf, true), patch_closed(&surf, false));
	if closed_u || closed_v {
		let windings: Vec<(i64, i64)> = rings.iter().map(|r| unwrap_ring(&mut uv, r, closed_u, closed_v)).collect();
		let wound: Vec<usize> = (0..rings.len()).filter(|&i| windings[i] != (0, 0)).collect();
		match wound.len() {
			// Every loop bounds a disk in the cover (seam-crossing/slit loops).
			0 => {}
			// A full-period band: exactly two rims wind ONE closed direction once,
			// in opposite senses, the outer bound being one of them (the untrimmed
			// closed patch, e.g. a NURBS tube wall bounded only by its two rims).
			2 if wound[0] == 0 => {
				let (wa, wb) = (windings[wound[0]], windings[wound[1]]);
				let in_u = wa.0 != 0;
				let unit = |w: (i64, i64)| (w.0.abs() <= 1 && w.1.abs() <= 1) && (w.0 == 0) != (w.1 == 0);
				if !(unit(wa) && unit(wb) && wa.0 + wb.0 == 0 && wa.1 + wb.1 == 0) {
					return Err(StepError::Unsupported(format!(
						"ADVANCED_FACE #{fid}: trimming loops wind {windings:?} periods around closed B-spline patch #{surface_ref} — only seam-crossing disk loops and a band between two opposite full-period rims are importable"
					)));
				}
				let rim_b = rings.remove(wound[1]);
				let rim_a = rings.remove(0);
				let merged = bridge_band_rings(&mut uv, &mut pts3, &rim_a, &rim_b, in_u);
				rings.insert(0, merged);
			}
			_ => {
				return Err(StepError::Unsupported(format!(
					"ADVANCED_FACE #{fid}: trimming loops wind {windings:?} periods around closed B-spline patch #{surface_ref} — only seam-crossing disk loops and a band between two opposite full-period rims are importable"
				)));
			}
		}
		// Each ring was unwrapped from its own first vertex, so hole rings may sit
		// whole periods away from the outer ring's cover window: translate them onto
		// it (mean-coordinate difference, rounded to whole periods). A hole that
		// still falls outside the outer loop fails the ear clip loudly below.
		let mean = |ring: &[usize], uv: &[DVec2]| {
			ring.iter().map(|&i| uv[i]).fold(DVec2::ZERO, |a, q| a + q) / ring.len() as f64
		};
		let outer_mean = mean(&rings[0], &uv);
		for ring in rings.iter().skip(1) {
			let d = outer_mean - mean(ring, &uv);
			let shift = DVec2::new(
				if closed_u { d.x.round() } else { 0.0 },
				if closed_v { d.y.round() } else { 0.0 },
			);
			if shift != DVec2::ZERO {
				for &i in ring {
					uv[i] += shift;
				}
			}
		}
	}
	// A single trim ring prefers the monotone sweep: it emits u-local (or, axes
	// swapped, v-local) triangles, so a slit ring's densely sampled rim runs never
	// fan long chords through the solid the way ear clipping a near-rectangle does
	// (each such fan chord would need to be refined away again). Non-monotone
	// single loops and every multi-ring (holed) trim fall back to the hole-bridging
	// ear clip.
	let monotone = (rings.len() == 1).then(|| {
		let ring = &rings[0];
		let ring_uv: Vec<DVec2> = ring.iter().map(|&i| uv[i]).collect();
		triangulate_monotone(&ring_uv).or_else(|_| triangulate_earclip(&ring_uv))
			.map(|ts| ts.into_iter().map(|t| [ring[t[0]], ring[t[1]], ring[t[2]]]).collect::<Vec<_>>())
			.or_else(|_| {
				// Swap the sweep axis: a v-monotone loop is u-monotone in the
				// transposed plane; transposition mirrors the winding, so swap it back.
				let swapped: Vec<DVec2> = ring_uv.iter().map(|q| DVec2::new(q.y, q.x)).collect();
				triangulate_monotone(&swapped).or_else(|_| triangulate_earclip(&swapped))
					.map(|ts| ts.into_iter().map(|t| [ring[t[0]], ring[t[2]], ring[t[1]]]).collect::<Vec<_>>())
			})
	});
	let mut tris = match monotone {
		Some(Ok(ts)) => ts,
		_ => triangulate_trim_rings(&uv, &rings).map_err(|m| StepError::Unsupported(format!("ADVANCED_FACE #{fid}: {m}")))?,
	};
	// Trim-loop segments are the watertight boundary: never split.
	let boundary: std::collections::HashSet<(usize, usize)> = rings
		.iter()
		.flat_map(|r| {
			let n = r.len();
			(0..n).map(move |i| edge_key(r[i], r[(i + 1) % n]))
		})
		.collect();
	// Interior refinement to the relative chordal tolerance: facet chords stay
	// within `PATCH_SAG_TOL` of the face's own scale from the exact surface — at
	// 1e-3 that matches the imported-conic fidelity contract (a 48-gon ring's
	// sagitta is 2.1e-3·r). Cover coordinates of a closed direction wrap back into
	// the fundamental domain for evaluation (`S` is periodic there by `patch_closed`).
	let wrap01 = |x: f64, closed: bool| if closed { x - x.floor() } else { x };
	let at = |q: DVec2| {
		(
			u_lo + (u_hi - u_lo) * wrap01(q.x, closed_u),
			v_lo + (v_hi - v_lo) * wrap01(q.y, closed_v),
		)
	};
	let eval = |q: DVec2| {
		let (u, v) = at(q);
		surf.point_at(u, v)
	};
	let scale = 1.0 + pts3.iter().map(|p| p.length()).fold(0.0_f64, f64::max);
	// Boundary handles keep their exact input positions (the weld); interior
	// handles are evaluated on the exact patch as refinement creates them.
	let mut pos3 = pts3.clone();
	refine_param_facets(&mut uv, &mut pos3, &mut tris, &boundary, eval, PATCH_SAG_TOL * scale)
		.map_err(|m| StepError::Unsupported(format!("ADVANCED_FACE #{fid}: {m}")))?;
	let pos = |h: usize| pos3[h];
	for t in &tris {
		let (pa, pb, pc) = (pos(t[0]), pos(t[1]), pos(t[2]));
		let (a, b, c) = (acc.intern(pa), acc.intern(pb), acc.intern(pc));
		if a == b || b == c || c == a {
			return Err(StepError::Topology(format!(
				"ADVANCED_FACE #{fid}: a patch facet degenerated to coincident vertices"
			)));
		}
		let centroid = (pa + pb + pc) / 3.0;
		let mut normal = (pb - pa).cross(pc - pa).normalize_or_zero();
		if normal.length_squared() < 0.5 {
			let (u, v) = at((uv[t[0]] + uv[t[1]] + uv[t[2]]) / 3.0);
			normal = surf.normal_at(u, v);
		}
		if normal.length_squared() < 0.5 {
			return Err(StepError::Topology(format!(
				"ADVANCED_FACE #{fid}: a patch facet has no usable normal (degenerate surface region)"
			)));
		}
		acc.faces.push(FaceLoops { loops: vec![vec![a, b, c]], surface: Surface::Plane { origin: centroid, normal } });
	}
	// Preserve the patch's NURBS identity alongside its chord facets (the analytic
	// [`Surface`] enum has no freeform variant): the exact rational surface plus the
	// verbatim trim rings — the sidecar [`import_step_freeform`] returns and
	// [`crate::step_export::export_step_freeform`] writes back out as a true
	// `B_SPLINE_SURFACE_WITH_KNOTS` face.
	acc.freeform.push(FreeformFace {
		surface: surf,
		rings: std::iter::once(outer_pts.to_vec()).chain(inner_loops.iter().cloned()).collect(),
	});
	Ok(())
}

/// Ear-clip a SIMPLE (non-self-intersecting) uv polygon into index triangles
/// wound like the input loop. The general fallback for boundaries the monotone
/// sweep refuses (e.g. a half-cylinder band whose rim carries a lug notch —
/// unwrapped, that rim dips in v and is not u-monotone). Diagonals of a simple
/// polygon stay inside it, so the periodic-seam guarantee of the sweep is
/// preserved: no triangle can jump the seam gap.
fn triangulate_earclip(uv: &[DVec2]) -> Result<Vec<[usize; 3]>, String> {
	let n = uv.len();
	if n < 3 {
		return Err("fewer than three boundary points".into());
	}
	let area2: f64 = (0..n).map(|i| uv[i].x * uv[(i + 1) % n].y - uv[(i + 1) % n].x * uv[i].y).sum();
	if area2 == 0.0 {
		return Err("zero-area parameter-space boundary".into());
	}
	let ccw = area2 > 0.0;
	let o = |a: DVec2, b: DVec2, c: DVec2| orient2d([a.x, a.y], [b.x, b.y], [c.x, c.y]);
	let inside = |p: DVec2, a: DVec2, b: DVec2, c: DVec2| {
		let (d1, d2, d3) = (o(p, a, b), o(p, b, c), o(p, c, a));
		let has_neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
		let has_pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
		!(has_neg && has_pos)
	};
	let mut idx: Vec<usize> = if ccw { (0..n).collect() } else { (0..n).rev().collect() };
	let mut out: Vec<[usize; 3]> = Vec::with_capacity(n - 2);
	let mut guard = 0usize;
	while idx.len() > 3 && guard < 40_000 {
		guard += 1;
		let m = idx.len();
		let mut clipped = false;
		for i in 0..m {
			let (ip, ic, inx) = (idx[(i + m - 1) % m], idx[i], idx[(i + 1) % m]);
			let (a, b, c) = (uv[ip], uv[ic], uv[inx]);
			if o(a, b, c) <= 0.0 {
				continue; // reflex or flat corner
			}
			if idx.iter().any(|&j| j != ip && j != ic && j != inx && inside(uv[j], a, b, c)) {
				continue;
			}
			out.push(if ccw { [ip, ic, inx] } else { [inx, ic, ip] });
			idx.remove(i);
			clipped = true;
			break;
		}
		if !clipped {
			return Err("ear clipping stalled on a degenerate uv boundary".into());
		}
	}
	if idx.len() == 3 {
		out.push(if ccw { [idx[0], idx[1], idx[2]] } else { [idx[2], idx[1], idx[0]] });
	}
	Ok(out)
}

/// Triangulate a simple u-monotone polygon (`uv` in loop order) into index triangles
/// wound like the input loop, via the classic two-chain sweep with a reflex stack
/// (de Berg et al. §3.3). Every triangle connects u-adjacent vertices, so the two
/// sides of a periodic seam (which sit a full period apart with ring vertices between
/// them) never join one triangle. Errors with a reason on non-monotone or degenerate
/// input rather than emitting garbage.
fn triangulate_monotone(uv: &[DVec2]) -> Result<Vec<[usize; 3]>, String> {
	let n = uv.len();
	if n < 3 {
		return Err("fewer than three boundary points".into());
	}
	let area2: f64 = (0..n)
		.map(|i| {
			let a = uv[i];
			let b = uv[(i + 1) % n];
			a.x * b.y - b.x * a.y
		})
		.sum();
	if area2 == 0.0 {
		return Err("zero-area parameter-space boundary".into());
	}
	// Work on a CCW view of the loop; flip the output winding back at the end.
	let ccw = area2 > 0.0;
	let at = |k: usize| if ccw { k } else { n - 1 - k };
	let p = |k: usize| uv[at(k)];
	let lex_less = |a: usize, b: usize| {
		let (pa, pb) = (p(a), p(b));
		pa.x.total_cmp(&pb.x).then(pa.y.total_cmp(&pb.y)) == std::cmp::Ordering::Less
	};
	let (mut lo, mut hi) = (0usize, 0usize);
	for k in 1..n {
		if lex_less(k, lo) {
			lo = k;
		}
		if lex_less(hi, k) {
			hi = k;
		}
	}
	// Chains: walking forward from the (lexicographic) min to the max is the LOWER
	// chain of a CCW polygon; walking backward, the upper. Verify monotonicity.
	let mut lower = vec![false; n];
	{
		let mut k = lo;
		while k != hi {
			lower[k] = true;
			let next = (k + 1) % n;
			if lex_less(next, k) {
				return Err("boundary is not u-monotone".into());
			}
			k = next;
		}
		let mut k = hi;
		while k != lo {
			let next = (k + 1) % n;
			if k != hi && lex_less(k, next) {
				return Err("boundary is not u-monotone".into());
			}
			k = next;
		}
	}
	// Merge the two (sorted) chains into one sweep order.
	let mut order: Vec<usize> = Vec::with_capacity(n);
	order.push(lo);
	{
		let (mut a, mut b) = ((lo + 1) % n, (lo + n - 1) % n);
		while a != hi || b != hi {
			if b == hi || (a != hi && lex_less(a, b)) {
				order.push(a);
				a = (a + 1) % n;
			} else {
				order.push(b);
				b = (b + n - 1) % n;
			}
		}
	}
	order.push(hi);
	for w in order.windows(2) {
		let (pa, pb) = (p(w[0]), p(w[1]));
		if pa.x == pb.x && pa.y == pb.y {
			return Err("coincident parameter-space boundary points".into());
		}
	}
	// Sweep with the reflex stack; orient every emitted triangle CCW (the pop order
	// alone does not fix the winding).
	let orient = |i: usize, j: usize, k: usize| orient2d([p(i).x, p(i).y], [p(j).x, p(j).y], [p(k).x, p(k).y]);
	let mut tris: Vec<[usize; 3]> = Vec::with_capacity(n - 2);
	let emit = |a: usize, b: usize, c: usize, tris: &mut Vec<[usize; 3]>| -> Result<(), String> {
		let o = orient(a, b, c);
		if o == 0.0 {
			return Err("degenerate (zero-height) parameter region".into());
		}
		tris.push(if o > 0.0 { [a, b, c] } else { [a, c, b] });
		Ok(())
	};
	let mut stack: Vec<usize> = vec![order[0], order[1]];
	for &vj in &order[2..n - 1] {
		let top = *stack.last().expect("the sweep stack is never empty");
		if lower[vj] != lower[top] {
			// Opposite chain: vj sees the whole stack; fan across it.
			while stack.len() > 1 {
				let v1 = stack.pop().expect("len > 1");
				let v2 = *stack.last().expect("len > 1 before pop");
				emit(vj, v1, v2, &mut tris)?;
			}
			stack.pop();
			stack.push(top);
		} else {
			// Same chain: clip while the diagonal to the next stack vertex stays inside
			// (a left turn seen from the lower chain, a right turn from the upper).
			let mut v1 = stack.pop().expect("the sweep stack is never empty");
			while let Some(&v2) = stack.last() {
				let o = orient(v2, v1, vj);
				let inside = if lower[vj] { o > 0.0 } else { o < 0.0 };
				if !inside {
					break;
				}
				emit(vj, v1, v2, &mut tris)?;
				v1 = v2;
				stack.pop();
			}
			stack.push(v1);
		}
		stack.push(vj);
	}
	// The final (max) vertex closes out every remaining stack diagonal.
	while stack.len() > 1 {
		let v1 = stack.pop().expect("len > 1");
		let v2 = *stack.last().expect("len > 1 before pop");
		emit(hi, v1, v2, &mut tris)?;
	}
	if tris.len() != n - 2 {
		return Err(format!("triangulation produced {} triangles for a {n}-gon", tris.len()));
	}
	Ok(tris
		.into_iter()
		.map(|t| {
			let m = [at(t[0]), at(t[1]), at(t[2])];
			if ccw {
				m
			} else {
				[m[0], m[2], m[1]]
			}
		})
		.collect())
}

/// Read one face bound's loop: the tessellated boundary (reversed when the bound's
/// orientation flag says so) and whether it carried any conic segments. Conic segments
/// are recorded by their (direction-independent) endpoint pair *before* any flip, for
/// analytic edge tagging after the solid is built.
fn read_bound_loop(
	imp: &Importer,
	loop_ref: u32,
	rev: bool,
	cache: &mut HashMap<u32, (Vec<DVec3>, Option<Curve>)>,
	conic_segments: &mut Vec<(DVec3, DVec3, Curve)>,
) -> Result<(Vec<DVec3>, bool), StepError> {
	let (mut pts, segs) = imp.loop_boundary(loop_ref, cache)?;
	let n = pts.len();
	let mut has_conic = false;
	for (i, seg) in segs.iter().enumerate() {
		if let Some(c) = seg {
			conic_segments.push((pts[i], pts[(i + 1) % n], *c));
			has_conic = true;
		}
	}
	if rev {
		pts.reverse();
	}
	dedup_ring(&mut pts);
	Ok((pts, has_conic))
}

/// Accumulates reconstructed faces (across one or more shells) into a shared
/// exact-position vertex pool, then builds the [`Solid`] — so every face set
/// (a whole file, or one `MANIFOLD_SOLID_BREP` of an assembly part) goes through
/// identical reconstruction.
#[derive(Default)]
struct FaceAccum {
	positions: Vec<DVec3>,
	index: HashMap<(u64, u64, u64), u32>,
	faces: Vec<FaceLoops>,
	conic_segments: Vec<(DVec3, DVec3, Curve)>,
	edge_cache: HashMap<u32, (Vec<DVec3>, Option<Curve>)>,
	/// The NURBS identity of every trimmed B-spline face reconstructed into `faces`
	/// (which carries chord facets only) — the sidecar [`import_step_freeform`] returns.
	freeform: Vec<FreeformFace>,
}

impl FaceAccum {
	fn intern(&mut self, p: DVec3) -> u32 {
		*self.index.entry(pos_key(p)).or_insert_with(|| {
			self.positions.push(p);
			(self.positions.len() - 1) as u32
		})
	}

	/// Build the solid from everything accumulated. Faces carry the producer's loop
	/// winding (outward CCW for a well-formed file), so `from_faces_multiloop` pairs
	/// the shared edges into a consistent 2-manifold directly — hole loops included.
	fn finish(self) -> Result<Solid, StepError> {
		if self.faces.is_empty() {
			return Err(StepError::Topology("no ADVANCED_FACE entities found".into()));
		}
		let mut solid = Solid::from_faces_multiloop(self.positions, self.faces);
		// Re-attach the analytic conic geometry to every boundary segment that carried
		// it, so a circular/elliptical edge round-trips as exact geometry, not a polyline.
		for (a, b, c) in self.conic_segments {
			if let (Some(&ia), Some(&ib)) = (self.index.get(&pos_key(a)), self.index.get(&pos_key(b))) {
				solid.set_edge_curve(VertexId(ia), VertexId(ib), c);
			}
		}
		Ok(solid)
	}
}

/// Reconstruct one `ADVANCED_FACE` into `acc` — one input face, or, for a periodic
/// wall / curved region / B-spline patch, a set of facets on its exact surface.
/// `flip` reverses every loop (an `ORIENTED_CLOSED_SHELL` `.F.` wrapper).
fn add_face(imp: &Importer, fid: u32, flip: bool, acc: &mut FaceAccum) -> Result<(), StepError> {
	let e = imp.get(fid)?;
	if e.name != "ADVANCED_FACE" {
		return Err(StepError::Reference(format!("#{fid} is {}, expected ADVANCED_FACE", e.name)));
	}
	// ADVANCED_FACE('', (#bound, …), #surface, same_sense)
	let bounds = e.args.iter().find_map(Value::as_list).ok_or_else(|| StepError::Parse(format!("ADVANCED_FACE #{fid} has no bound list")))?;
	// The surface is the last bare reference after the bound list.
	let surface_ref = e
		.args
		.iter()
		.filter_map(Value::as_ref)
		.next_back()
		.ok_or_else(|| StepError::Parse(format!("ADVANCED_FACE #{fid} has no surface")))?;
	// The face's same-sense flag (used to orient slit-bounded full-periodic regions,
	// where the boundary loop itself encloses no signed area). `flip` inverts it.
	let face_same_sense = last_enum(e).map(|s| s == "T").unwrap_or(true) ^ flip;
	// A B-spline surface face is tessellated on the exact patch (see below); other
	// unsupported surfaces stay loud.
	let surface = match imp.surface(surface_ref) {
		Ok(s) => Some(s),
		Err(StepError::Unsupported(_)) if is_bspline_surface(imp.get(surface_ref)?) => None,
		Err(err) => return Err(err),
	};

	// Partition the bounds: one outer loop + inner (hole) loops.
	let mut outer: Option<(u32, bool)> = None;
	let mut inner: Vec<(u32, bool)> = Vec::new();
	for b in bounds {
		let bid = b.as_ref().ok_or_else(|| StepError::Parse("face bound is not a reference".into()))?;
		let be = imp.get(bid)?;
		let loop_ref = be.args.iter().find_map(Value::as_ref).ok_or_else(|| StepError::Parse(format!("#{bid} bound has no loop")))?;
		// A bound's `.F.` orientation flag means its loop is stored reversed.
		let rev = !last_enum(be).map(|s| s == "T").unwrap_or(true) ^ flip;
		match be.name.as_str() {
			"FACE_OUTER_BOUND" => {
				if outer.replace((loop_ref, rev)).is_some() {
					return Err(StepError::Topology(format!("ADVANCED_FACE #{fid} has two FACE_OUTER_BOUNDs")));
				}
			}
			"FACE_BOUND" => inner.push((loop_ref, rev)),
			other => return Err(StepError::Unsupported(format!("face bound #{bid} of type {other}"))),
		}
	}
	// Without an explicit outer bound a single loop is the outer; several unmarked
	// loops are ambiguous and refused rather than guessed.
	let (outer_ref, outer_rev) = match (outer, inner.len()) {
		(Some(o), _) => o,
		(None, 1) => inner.pop().expect("one element was just checked"),
		(None, count) => {
			return Err(StepError::Unsupported(format!(
				"ADVANCED_FACE #{fid} has {count} FACE_BOUNDs but no FACE_OUTER_BOUND marking the outer loop"
			)))
		}
	};

	let (outer_pts, outer_conic) = read_bound_loop(imp, outer_ref, outer_rev, &mut acc.edge_cache, &mut acc.conic_segments)?;
	if outer_pts.len() < 3 {
		if outer_conic {
			// Even tessellated, the loop collapsed (e.g. a lens of two sub-90° arcs
			// whose chords coincide): refuse loudly rather than emit a degenerate face.
			return Err(StepError::Unsupported(format!(
				"ADVANCED_FACE #{fid}: an arc-bounded loop collapsed to fewer than 3 boundary points"
			)));
		}
		return Ok(()); // genuinely degenerate (zero-length) planar loop — skip, as exporters do
	}
	let mut inner_loops: Vec<Vec<DVec3>> = Vec::new();
	for (loop_ref, rev) in inner {
		let (pts, conic) = read_bound_loop(imp, loop_ref, rev, &mut acc.edge_cache, &mut acc.conic_segments)?;
		if pts.len() < 3 {
			if conic {
				return Err(StepError::Unsupported(format!(
					"ADVANCED_FACE #{fid}: an arc-bounded inner loop collapsed to fewer than 3 boundary points"
				)));
			}
			continue; // degenerate sliver hole — bounds nothing
		}
		inner_loops.push(pts);
	}

	let Some(surface) = surface else {
		// A trimmed B_SPLINE_SURFACE face: tessellated on the exact patch.
		return add_bspline_face(imp, fid, surface_ref, &outer_pts, &inner_loops, acc);
	};

	match surface {
		Surface::Plane { .. } => {
			// Planar faces carry their inner (hole) loops directly — the kernel's
			// multi-loop face input, as `extrude_with_holes` builds it.
			let mut loops: Vec<Vec<u32>> = Vec::with_capacity(1 + inner_loops.len());
			loops.push(outer_pts.iter().map(|&p| acc.intern(p)).collect());
			for lp in &inner_loops {
				loops.push(lp.iter().map(|&p| acc.intern(p)).collect());
			}
			acc.faces.push(FaceLoops { loops, surface });
		}
		curved => {
			if !inner_loops.is_empty() {
				return Err(StepError::Unsupported(format!(
					"ADVANCED_FACE #{fid}: inner loops on a curved analytic face are not importable yet — only planar and B-spline faces may carry holes"
				)));
			}
			if outer_pts.len() <= 4 || is_chord_facet(&outer_pts, &curved) {
				// A native chord facet — ≤4 vertices (this kernel's own cylinder/
				// sphere bands) or a flat, narrow-span polygon on the surface (a
				// boolean-recovered band). Kept as ONE face, matching the
				// tessellator's flat-chord semantics and keeping own-export
				// round-trips exact.
				let idx: Vec<u32> = outer_pts.iter().map(|&p| acc.intern(p)).collect();
				acc.faces.push(FaceLoops { loops: vec![idx], surface: curved });
			} else {
				match curved {
					Surface::Cylinder { .. } | Surface::Cone { .. } => {
						// The seam-aware unwrap first, then VERIFY it against the parameter
						// chart. The oracle is FLUX, not area: two triangulations of the
						// SAME boundary ring differ in flux by exactly the volume enclosed
						// between them (divergence theorem), so a strip that folds back on
						// itself — what a mesher-jagged merged face's non-monotone unwrap
						// produces — is caught even when its total area looks right.
						// Measured: a recovered implicit cylinder's wall re-imported 37.6%
						// light through the unverified strip. An ordinary chord band (what
						// the exporter coalesces a builder wall into) matches the chart to
						// well under the bar and keeps its exact reconstruction.
						let charted = general_curved_region(&outer_pts, &curved);
						let strip = split_periodic_face(&outer_pts, &curved, fid);
						let strip_ok = match (&strip, &charted) {
							(Ok(tris), Some((extras, ctris))) => {
								let pos = |h: usize| if h < outer_pts.len() { outer_pts[h] } else { extras[h - outer_pts.len()] };
								// Flux about a local anchor (the surface's own origin) keeps
								// the terms at model scale.
								let anchor = match curved {
									Surface::Cylinder { origin, .. } => origin,
									Surface::Cone { apex, .. } => apex,
									_ => DVec3::ZERO,
								};
								let flux = |ts: &[[usize; 3]], p: &dyn Fn(usize) -> DVec3| -> f64 {
									ts.iter()
										.map(|t| {
											let (a, b, c) = (p(t[0]) - anchor, p(t[1]) - anchor, p(t[2]) - anchor);
											a.dot(b.cross(c)) / 6.0
										})
										.sum()
								};
								let scale = outer_pts.iter().map(|p| (*p - anchor).length()).fold(0.0_f64, f64::max).max(1e-9);
								let (fs, fc) = (flux(tris, &|h| outer_pts[h]), flux(ctris, &pos));
								// Both windings follow the same input ring, so a healthy
								// strip agrees with the chart to a chord sagitta.
								(fs - fc).abs() <= 0.02 * fc.abs().max(scale.powi(3) * 1e-3)
							}
							(Ok(_), None) => true,
							(Err(_), _) => false,
						};
						if !strip_ok {
							let (extras, tris) = charted.expect("a failed strip check implies a chart triangulation exists");
							let pos = |h: usize| if h < outer_pts.len() { outer_pts[h] } else { extras[h - outer_pts.len()] };
							for t in tris {
								let (a, b, c) = (acc.intern(pos(t[0])), acc.intern(pos(t[1])), acc.intern(pos(t[2])));
								if a == b || b == c || c == a {
									return Err(StepError::Topology(format!(
										"ADVANCED_FACE #{fid}: a charted facet degenerated onto a repeated boundary point"
									)));
								}
								acc.faces.push(FaceLoops { loops: vec![vec![a, b, c]], surface: curved });
							}
							return Ok(());
						}
						let tris = strip?;
						let idx: Vec<u32> = outer_pts.iter().map(|&p| acc.intern(p)).collect();
						for t in tris {
							let (a, b, c) = (idx[t[0]], idx[t[1]], idx[t[2]]);
							if a == b || b == c || c == a {
								// A facet joining both copies of the seam vertex would be a
								// zero-width sliver; monotone sweep cannot produce one unless
								// the input was degenerate.
								return Err(StepError::Topology(format!(
									"ADVANCED_FACE #{fid}: a split facet degenerated onto the seam"
								)));
							}
							acc.faces.push(FaceLoops { loops: vec![vec![a, b, c]], surface: curved });
						}
					}
					Surface::Sphere { .. } | Surface::Torus { .. } => {
						// A periodic / pole-spanning sphere or torus region: resampled into a
						// ring grid on the exact surface (see `resample_periodic_region`).
						// Sub-periodic regions took the chart path above.
						let axis = match curved {
							Surface::Torus { axis, .. } => axis,
							_ => imp.surface_axis(surface_ref)?,
						};
						let (extras, tris) = resample_periodic_region(&outer_pts, &curved, axis, face_same_sense, fid)?;
						// Intern lazily, per referenced handle: seam (slit) boundary points
						// are interior to the face and may legitimately go unused — an
						// eagerly interned copy would become an isolated vertex and corrupt
						// the Euler characteristic.
						let pos = |h: usize| if h < outer_pts.len() { outer_pts[h] } else { extras[h - outer_pts.len()] };
						for t in tris {
							let (a, b, c) = (acc.intern(pos(t[0])), acc.intern(pos(t[1])), acc.intern(pos(t[2])));
							if a == b || b == c || c == a {
								return Err(StepError::Topology(format!(
									"ADVANCED_FACE #{fid}: a resampled facet degenerated onto the seam"
								)));
							}
							acc.faces.push(FaceLoops { loops: vec![vec![a, b, c]], surface: curved });
						}
					}
					Surface::Plane { .. } => unreachable!("planar faces are handled above"),
				}
			}
		}
	}
	Ok(())
}

/// Import a STEP physical-file string and reconstruct a B-rep [`Solid`].
///
/// Faces keep their exact analytic [`Surface`] tags and conic edges their analytic
/// [`Curve`]s. Arc-bounded faces (a cap bounded by ONE full-circle edge), periodic
/// cylinder/cone walls (seam edge + circular rims), pole-spanning / fully periodic
/// sphere and torus regions, trimmed B-spline patches and planar faces with inner
/// (hole) loops are reconstructed per the module-level support matrix; anything
/// outside it is a loud [`StepError::Unsupported`], never a silent drop. Shared
/// vertices are merged by exact position so adjacent faces pair their edges into a
/// watertight 2-manifold. Faces come from every `MANIFOLD_SOLID_BREP` (in entity-id
/// order — a multi-solid file imports as one [`Solid`] with several shells), falling
/// back to all `ADVANCED_FACE`s for bare fragments.
///
/// ```
/// use kernel_brep::{export_step, import_step, cuboid, volume};
/// use kernel_brep::math::DVec3;
/// let box_ = cuboid(DVec3::splat(-1.0), DVec3::splat(1.0));
/// let step = export_step(&box_, "box");
/// let back = import_step(&step).unwrap();
/// assert!((volume(&back).abs() - 8.0).abs() < 1e-9);
/// ```
pub fn import_step(text: &str) -> Result<Solid, StepError> {
	Ok(import_step_freeform(text)?.0)
}

/// [`import_step`] plus the **NURBS sidecar**: every trimmed `B_SPLINE_SURFACE`
/// face's exact rational patch and verbatim trim rings as a [`FreeformFace`], in
/// face order. The [`Solid`] itself carries only the chord facets (the analytic
/// [`Surface`] enum has no freeform variant); the sidecar is what preserves the
/// patches' NURBS identity so [`crate::step_export::export_step_freeform`] can
/// re-export them as true `B_SPLINE_SURFACE_WITH_KNOTS` faces — the writing half
/// of NURBS interchange. Files without B-spline faces return an empty sidecar.
pub fn import_step_freeform(text: &str) -> Result<(Solid, Vec<FreeformFace>), StepError> {
	let ents = parse(text)?;
	let imp = Importer { ents: &ents };

	// Deterministic face collection: solid-model shells first, bare faces as fallback.
	let mut face_list: Vec<(u32, bool)> = Vec::new();
	for (_, faces) in imp.brep_face_sets()? {
		face_list.extend(faces);
	}
	if face_list.is_empty() {
		face_list = imp.all_face_ids();
	}

	let mut acc = FaceAccum::default();
	for (fid, flip) in face_list {
		add_face(&imp, fid, flip, &mut acc)?;
	}
	let freeform = std::mem::take(&mut acc.freeform);
	Ok((acc.finish()?, freeform))
}

// --- Assemblies ----------------------------------------------------------------

/// Maximum component-tree depth walked by [`import_step_assembly`] (a cycle in a
/// malformed NAUO/MAPPED_ITEM graph errors loudly instead of recursing forever).
const ASSEMBLY_MAX_DEPTH: usize = 64;

/// Assembly-structure resolver layered over the entity graph: product names,
/// product-definition → shape-representation links, NAUO child relations with their
/// `ITEM_DEFINED_TRANSFORMATION` placements, and `MAPPED_ITEM` instancing.
struct AssemblyGraph<'a> {
	imp: &'a Importer<'a>,
	/// `PRODUCT_DEFINITION` id → `SHAPE_REPRESENTATION`-family id (via
	/// `PRODUCT_DEFINITION_SHAPE` + `SHAPE_DEFINITION_REPRESENTATION`).
	shape_rep: HashMap<u32, u32>,
	/// NAUO id → `(parent PRODUCT_DEFINITION, child PRODUCT_DEFINITION)`.
	nauo: Vec<(u32, (u32, u32))>,
	/// NAUO id → child→parent placement, from the `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`.
	nauo_transform: HashMap<u32, DAffine3>,
	/// Solids already reconstructed, keyed by their representation id (instances share).
	solid_cache: HashMap<u32, Solid>,
}

impl<'a> AssemblyGraph<'a> {
	/// The product name of a `PRODUCT_DEFINITION`: its formation's product's name
	/// (the first string argument of the `PRODUCT`).
	fn product_name(&self, pd: u32) -> Result<String, StepError> {
		let pd_ent = self.imp.get(pd)?;
		let formation = pd_ent
			.args
			.iter()
			.find_map(Value::as_ref)
			.ok_or_else(|| StepError::Parse(format!("#{pd} PRODUCT_DEFINITION has no formation")))?;
		let product = self
			.imp
			.get(formation)?
			.args
			.iter()
			.find_map(Value::as_ref)
			.ok_or_else(|| StepError::Parse(format!("#{formation} PRODUCT_DEFINITION_FORMATION has no product")))?;
		let name = self.imp.get(product)?.args.iter().find_map(|v| match v {
			Value::Str(s) => Some(s.clone()),
			_ => None,
		});
		Ok(name.unwrap_or_else(|| format!("product #{product}")))
	}

	/// Reconstruct (or fetch from cache) the [`Solid`] of one representation: every
	/// `MANIFOLD_SOLID_BREP`/`BREP_WITH_VOIDS` in its item list, rebuilt through the
	/// same face accumulator as [`import_step`]. `None` when the representation
	/// carries no breps (a pure-placement assembly root).
	fn rep_solid(&mut self, rep: u32) -> Result<Option<Solid>, StepError> {
		if let Some(s) = self.solid_cache.get(&rep) {
			return Ok(Some(s.clone()));
		}
		let mut brep_ids: Vec<u32> = self
			.rep_items(rep)?
			.into_iter()
			.filter(|&id| {
				self.imp
					.get(id)
					.map(|e| e.name == "MANIFOLD_SOLID_BREP" || e.name == "BREP_WITH_VOIDS")
					.unwrap_or(false)
			})
			.collect();
		brep_ids.sort_unstable();
		if brep_ids.is_empty() {
			return Ok(None);
		}
		let mut acc = FaceAccum::default();
		for brep in brep_ids {
			let e = self.imp.get(brep)?;
			let outer = e
				.args
				.iter()
				.find_map(Value::as_ref)
				.ok_or_else(|| StepError::Parse(format!("#{brep} {} has no outer shell", e.name)))?;
			let mut faces = self.imp.shell_faces(outer, false)?;
			if e.name == "BREP_WITH_VOIDS" {
				let voids = e
					.args
					.iter()
					.find_map(Value::as_list)
					.ok_or_else(|| StepError::Parse(format!("#{brep} BREP_WITH_VOIDS has no void list")))?;
				for v in voids {
					let vid = v.as_ref().ok_or_else(|| StepError::Parse(format!("#{brep} void shell is not a reference")))?;
					faces.extend(self.imp.shell_faces(vid, false)?);
				}
			}
			for (fid, flip) in faces {
				add_face(self.imp, fid, flip, &mut acc)?;
			}
		}
		let solid = acc.finish()?;
		self.solid_cache.insert(rep, solid.clone());
		Ok(Some(solid))
	}

	/// The item-reference list of a representation entity (the second list argument,
	/// after the name).
	fn rep_items(&self, rep: u32) -> Result<Vec<u32>, StepError> {
		let e = self.imp.get(rep)?;
		Ok(e.args
			.iter()
			.find_map(Value::as_list)
			.ok_or_else(|| StepError::Parse(format!("#{rep} {} has no item list", e.name)))?
			.iter()
			.filter_map(Value::as_ref)
			.collect())
	}

	/// The `MAPPED_ITEM`s of a representation: `(source representation, placement)`
	/// pairs, each placing the source's geometry at the mapped target frame relative
	/// to the map's origin frame.
	fn mapped_items(&self, rep: u32) -> Result<Vec<(u32, DAffine3)>, StepError> {
		let mut out = Vec::new();
		for id in self.rep_items(rep)? {
			let e = self.imp.get(id)?;
			if e.name != "MAPPED_ITEM" {
				continue;
			}
			// MAPPED_ITEM('', #REPRESENTATION_MAP, #target_placement)
			let refs: Vec<u32> = e.args.iter().filter_map(Value::as_ref).collect();
			if refs.len() < 2 {
				return Err(StepError::Parse(format!("#{id} MAPPED_ITEM needs a map and a target placement")));
			}
			let map = self.imp.get(refs[0])?;
			if map.name != "REPRESENTATION_MAP" {
				return Err(StepError::Reference(format!("#{} is {}, expected REPRESENTATION_MAP", refs[0], map.name)));
			}
			// REPRESENTATION_MAP(#origin_placement, #source_representation)
			let map_refs: Vec<u32> = map.args.iter().filter_map(Value::as_ref).collect();
			if map_refs.len() < 2 {
				return Err(StepError::Parse(format!("#{} REPRESENTATION_MAP needs an origin and a representation", refs[0])));
			}
			let origin = placement_affine(self.imp, map_refs[0])?;
			let target = placement_affine(self.imp, refs[1])?;
			out.push((map_refs[1], target * origin.inverse()));
		}
		Ok(out)
	}

	/// Emit one component per brep-bearing representation reachable from `rep`
	/// (its own breps, plus nested `MAPPED_ITEM` instances), placed by `at`.
	fn emit_rep(
		&mut self,
		name: &str,
		rep: u32,
		at: DAffine3,
		depth: usize,
		out: &mut Vec<(String, Solid, DAffine3)>,
	) -> Result<(), StepError> {
		if depth > ASSEMBLY_MAX_DEPTH {
			return Err(StepError::Topology(format!(
				"assembly mapping nests deeper than {ASSEMBLY_MAX_DEPTH} — the MAPPED_ITEM graph has a cycle"
			)));
		}
		if let Some(solid) = self.rep_solid(rep)? {
			out.push((name.to_string(), solid, at));
		}
		for (src, t) in self.mapped_items(rep)? {
			let src_name = self
				.imp
				.get(src)?
				.args
				.iter()
				.find_map(|v| match v {
					Value::Str(s) if !s.is_empty() => Some(s.clone()),
					_ => None,
				})
				.unwrap_or_else(|| name.to_string());
			self.emit_rep(&src_name, src, at * t, depth + 1, out)?;
		}
		Ok(())
	}

	/// Flatten the component tree under `pd`, accumulating placements: leaves (no
	/// child NAUOs) emit their representation's solid; assembly nodes recurse into
	/// their children, and any geometry carried by the node itself is emitted too.
	fn walk(&mut self, pd: u32, at: DAffine3, depth: usize, out: &mut Vec<(String, Solid, DAffine3)>) -> Result<(), StepError> {
		if depth > ASSEMBLY_MAX_DEPTH {
			return Err(StepError::Topology(format!(
				"assembly tree nests deeper than {ASSEMBLY_MAX_DEPTH} — the NAUO graph has a cycle"
			)));
		}
		let name = self.product_name(pd)?;
		if let Some(&rep) = self.shape_rep.get(&pd) {
			self.emit_rep(&name, rep, at, depth, out)?;
		}
		let children: Vec<(u32, u32)> = self
			.nauo
			.iter()
			.filter(|(_, (parent, _))| *parent == pd)
			.map(|&(id, (_, child))| (id, child))
			.collect();
		for (nauo_id, child) in children {
			let t = self.nauo_transform.get(&nauo_id).copied().unwrap_or(DAffine3::IDENTITY);
			self.walk(child, at * t, depth + 1, out)?;
		}
		Ok(())
	}
}

/// The affine frame of an `AXIS2_PLACEMENT_3D`: columns `(x, y = axis × x, axis)`
/// with the translation at the placement location — the map from the local frame
/// into world coordinates.
fn placement_affine(imp: &Importer, id: u32) -> Result<DAffine3, StepError> {
	let (origin, axis, x, y) = imp.frame(id)?;
	Ok(DAffine3::from_mat3_translation(DMat3::from_cols(x, y, axis), origin))
}

/// The child→parent placement of one NAUO, from its `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`:
/// the CDSR's relationship complex carries an `ITEM_DEFINED_TRANSFORMATION` mapping
/// frame 1 into frame 2 between `REPRESENTATION_RELATIONSHIP` reps `(rep_1, rep_2)`.
/// When `rep_1` is the CHILD's representation the placement is `frame2 ∘ frame1⁻¹`;
/// writers that store the pair reversed get the inverse. A NAUO with no CDSR places
/// its child at the identity.
fn nauo_placement(imp: &Importer, nauo: u32, child_rep: Option<u32>) -> Result<Option<DAffine3>, StepError> {
	// Entity scans run in ascending-id order so a (malformed) file with duplicate
	// records still resolves deterministically across runs.
	let ids_of = |name: &str| -> Vec<u32> {
		let mut ids: Vec<u32> = imp.ents.iter().filter(|(_, e)| e.name == name).map(|(&id, _)| id).collect();
		ids.sort_unstable();
		ids
	};
	// Find the PRODUCT_DEFINITION_SHAPE that describes this NAUO…
	let pds_of_nauo = ids_of("PRODUCT_DEFINITION_SHAPE").into_iter().find(|&id| {
		imp.ents[&id].args.iter().filter_map(Value::as_ref).any(|r| r == nauo)
	});
	let Some(pds) = pds_of_nauo else { return Ok(None) };
	// …then the CONTEXT_DEPENDENT_SHAPE_REPRESENTATION pointing at that shape aspect.
	for id in ids_of("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION") {
		let e = &imp.ents[&id];
		let refs: Vec<u32> = e.args.iter().filter_map(Value::as_ref).collect();
		if refs.len() < 2 || refs[1] != pds {
			continue;
		}
		// refs[0] is the (usually _COMPLEX) representation relationship.
		let rel = imp.get(refs[0])?;
		let (rel_args, idt_ref) = match rel.name.as_str() {
			"_COMPLEX" => {
				let rr = complex_part(&rel.args, "REPRESENTATION_RELATIONSHIP")
					.ok_or_else(|| StepError::Parse(format!("#{} relationship complex has no REPRESENTATION_RELATIONSHIP", refs[0])))?;
				let rrwt = complex_part(&rel.args, "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION")
					.ok_or_else(|| StepError::Parse(format!("#{} relationship complex has no transformation record", refs[0])))?;
				(rr, rrwt.iter().find_map(Value::as_ref))
			}
			"SHAPE_REPRESENTATION_RELATIONSHIP" | "REPRESENTATION_RELATIONSHIP" => (rel.args.as_slice(), None),
			other => {
				return Err(StepError::Unsupported(format!(
					"NAUO #{nauo}: relationship #{} of type {other}",
					refs[0]
				)))
			}
		};
		let Some(idt) = idt_ref else {
			return Ok(Some(DAffine3::IDENTITY)); // an untransformed relationship
		};
		let idt_ent = imp.get(idt)?;
		if idt_ent.name != "ITEM_DEFINED_TRANSFORMATION" {
			return Err(StepError::Unsupported(format!(
				"NAUO #{nauo}: transformation #{idt} of type {} (only ITEM_DEFINED_TRANSFORMATION is importable)",
				idt_ent.name
			)));
		}
		let frames: Vec<u32> = idt_ent.args.iter().filter_map(Value::as_ref).collect();
		if frames.len() < 2 {
			return Err(StepError::Parse(format!("#{idt} ITEM_DEFINED_TRANSFORMATION needs two placements")));
		}
		let f1 = placement_affine(imp, frames[0])?;
		let f2 = placement_affine(imp, frames[1])?;
		let reps: Vec<u32> = rel_args.iter().filter_map(Value::as_ref).collect();
		// rep_1 → rep_2 is frame1 → frame2; orient it child → parent.
		let child_first = match (child_rep, reps.first()) {
			(Some(c), Some(&r1)) => r1 == c,
			_ => true,
		};
		return Ok(Some(if child_first { f2 * f1.inverse() } else { f1 * f2.inverse() }));
	}
	Ok(None)
}

/// Import the **assembly structure** of a STEP file: the flattened component
/// instances as `(product name, part solid, placement)` triples.
///
/// Components come from `NEXT_ASSEMBLY_USAGE_OCCURRENCE` relations (the AP214
/// product tree), each instance placed by its `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`'s
/// `ITEM_DEFINED_TRANSFORMATION` — placements accumulate down nested
/// sub-assemblies, and instances of one part share the same reconstructed geometry
/// (the solid is rebuilt per instance from one cached reconstruction). Files that
/// instance geometry with `MAPPED_ITEM`/`REPRESENTATION_MAP` instead are flattened
/// the same way. A file with NO assembly structure degrades gracefully: every
/// brep-bearing product (or, failing that, the whole file) is returned as a single
/// component at the identity placement, so the function is total over valid
/// part/assembly files. Assembly NODES legitimately carry no geometry of their own
/// (their representation holds only a placement) and contribute no component —
/// only an entire tree without a single brep is an error. Geometry reconstruction
/// is exactly [`import_step`]'s — including every loud [`StepError::Unsupported`]
/// in the module support matrix.
///
/// The placement is a [`DAffine3`] mapping the part's local frame into assembly
/// space; `placement.transform_point3(p)` places a local point.
pub fn import_step_assembly(text: &str) -> Result<Vec<(String, Solid, DAffine3)>, StepError> {
	let ents = parse(text)?;
	let imp = Importer { ents: &ents };

	// PRODUCT_DEFINITION → representation links (SHAPE_DEFINITION_REPRESENTATION over
	// PRODUCT_DEFINITION_SHAPE), skipping shape aspects that describe NAUOs. The scan
	// runs in ascending-id order so duplicate links resolve deterministically.
	let mut sdr_ids: Vec<u32> = ents
		.iter()
		.filter(|(_, e)| e.name == "SHAPE_DEFINITION_REPRESENTATION")
		.map(|(&id, _)| id)
		.collect();
	sdr_ids.sort_unstable();
	let mut shape_rep: HashMap<u32, u32> = HashMap::new();
	for id in sdr_ids {
		let e = &ents[&id];
		let refs: Vec<u32> = e.args.iter().filter_map(Value::as_ref).collect();
		if refs.len() < 2 {
			continue;
		}
		let (pds, rep) = (refs[0], refs[1]);
		let Ok(pds_ent) = imp.get(pds) else { continue };
		if pds_ent.name != "PRODUCT_DEFINITION_SHAPE" {
			continue;
		}
		if let Some(target) = pds_ent.args.iter().find_map(Value::as_ref) {
			if imp.get(target).map(|t| t.name == "PRODUCT_DEFINITION").unwrap_or(false) {
				shape_rep.insert(target, rep);
			}
		}
	}

	// NAUO relations, in entity-id order for determinism.
	let mut nauo: Vec<(u32, (u32, u32))> = Vec::new();
	for (&id, e) in ents.iter() {
		if e.name == "NEXT_ASSEMBLY_USAGE_OCCURRENCE" {
			let refs: Vec<u32> = e.args.iter().filter_map(Value::as_ref).collect();
			if refs.len() < 2 {
				return Err(StepError::Parse(format!("#{id} NEXT_ASSEMBLY_USAGE_OCCURRENCE needs parent and child")));
			}
			nauo.push((id, (refs[0], refs[1])));
		}
	}
	nauo.sort_unstable_by_key(|&(id, _)| id);

	let mut graph = AssemblyGraph { imp: &imp, shape_rep, nauo, nauo_transform: HashMap::new(), solid_cache: HashMap::new() };
	for (id, (_, child)) in graph.nauo.clone() {
		let child_rep = graph.shape_rep.get(&child).copied();
		if let Some(t) = nauo_placement(&imp, id, child_rep)? {
			graph.nauo_transform.insert(id, t);
		}
	}

	let mut out: Vec<(String, Solid, DAffine3)> = Vec::new();
	if graph.nauo.is_empty() {
		// No assembly tree: emit every brep-bearing product directly (mapped items
		// included), falling back to the whole file as one anonymous component.
		let mut pds: Vec<u32> = graph.shape_rep.keys().copied().collect();
		pds.sort_unstable();
		for pd in pds {
			graph.walk(pd, DAffine3::IDENTITY, 0, &mut out)?;
		}
		if out.is_empty() {
			out.push(("solid".to_string(), import_step(text)?, DAffine3::IDENTITY));
		}
		return Ok(out);
	}

	// Roots: products that parent at least one NAUO but are never a child.
	let children: std::collections::HashSet<u32> = graph.nauo.iter().map(|&(_, (_, c))| c).collect();
	let mut roots: Vec<u32> = graph
		.nauo
		.iter()
		.map(|&(_, (p, _))| p)
		.filter(|p| !children.contains(p))
		.collect();
	roots.sort_unstable();
	roots.dedup();
	if roots.is_empty() {
		return Err(StepError::Topology("the NAUO graph has no root (every product is someone's child — a cycle)".into()));
	}
	for root in roots {
		graph.walk(root, DAffine3::IDENTITY, 0, &mut out)?;
	}
	if out.is_empty() {
		return Err(StepError::Topology("the assembly tree reached no brep-bearing component".into()));
	}
	Ok(out)
}
