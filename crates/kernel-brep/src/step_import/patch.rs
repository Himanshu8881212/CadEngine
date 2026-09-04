// Copyright (c) LMCAD. Licensed under the MIT License.

//! [`ParamPatch`] — the parameter-space view a trimmed face is tessellated on,
//! with the two implementations (an exact NURBS patch and an analytic quadric)
//! and the pole/seam bookkeeping that makes a closed direction well behaved.

use kernel_core::math::{DVec2, DVec3};

use crate::geom::{perp_basis, Surface};
use crate::nurbs::NurbsSurface;

use super::edges::FULL_TURN_SEGMENTS;
use super::patch_tess::{patch_closed, patch_seed_grid, PATCH_PROJECT_TOL, PATCH_SAG_TOL, PATCH_SEED_GRID};

/// A boundary vertex located on a parameter patch ([`ParamPatch::locate`]).
pub(super) struct Located {
	/// Normalised patch coordinates: `[0,1]²` on a B-spline patch; one period = 1
	/// along a closed analytic direction.
	pub(crate) uv: DVec2,
	/// Whether `uv.x` is meaningful — `false` at a sphere pole / cone apex, where
	/// the angle is undefined and is interpolated from the loop neighbours.
	pub(crate) u_defined: bool,
	/// The distance (mm) the vertex sat off the patch when it was accepted only
	/// under the file's uncertainty allowance — a snap the tolerant receipt
	/// reports as a repair. `None` when it lay on the patch to the strict
	/// tolerance.
	pub(crate) snapped: Option<f64>,
}

/// A face's supporting surface seen as a **normalised parameter patch** — the
/// abstraction the trimmed-face tessellation ([`add_patch_face`]) works on, so a
/// trimmed B-spline patch ([`NurbsPatch`]) and a trimmed analytic quadric
/// ([`AnalyticPatch`]) share ONE loop-unwrapping, hole-bridging, triangulation
/// and on-surface refinement path: holes on curved faces, seam crossings, slit
/// seams, two-rim bands and one-rim caps.
pub(super) trait ParamPatch {
	/// Locate a boundary vertex on the patch; `Err(reason)` when it is not on it.
	fn locate(&self, p: DVec3) -> Result<Located, String>;
	/// The exact surface point at normalised `uv` (closed directions wrap).
	fn point(&self, uv: DVec2) -> DVec3;
	/// The surface's own unit normal at `uv` (its natural orientation, NOT the
	/// face's `same_sense`-adjusted one).
	fn normal(&self, uv: DVec2) -> DVec3;
	/// Which normalised directions are closed (periodic with period 1).
	fn closed(&self) -> (bool, bool);
	/// The `v` of the degenerate pole row that closes a one-rim cap on the `north`
	/// (`+v`) or south side, for surfaces that have poles (a sphere); `None` else.
	fn pole_v(&self, north: bool) -> Option<f64>;
	/// Absolute chordal tolerance (mm) for the interior refinement.
	fn sag_tol(&self, boundary: &[DVec3]) -> f64;
	/// Millimetres per normalised chart unit, per direction — the metric the
	/// batched refinement measures edge lengths and Delaunay quality in.
	fn chart_scale(&self) -> DVec2;
	/// The [`Surface`] tag a chord facet with this centroid/normal carries.
	fn facet_surface(&self, centroid: DVec3, normal: DVec3) -> Surface;
	/// The exact NURBS identity to keep in the freeform sidecar, if any.
	fn nurbs(&self) -> Option<&NurbsSurface>;
	/// Human label for error messages.
	fn label(&self) -> String;
}

/// Wrap a normalised coordinate into `[0, 1)` when its direction is closed.
fn wrap01(x: f64, closed: bool) -> f64 {
	if closed {
		x - x.floor()
	} else {
		x
	}
}

/// A trimmed `B_SPLINE_SURFACE_WITH_KNOTS` patch as a [`ParamPatch`].
pub(super) struct NurbsPatch {
	surf: NurbsSurface,
	grid: Vec<(DVec2, DVec3)>,
	id: u32,
	closed_u: bool,
	closed_v: bool,
	/// Absolute distance (mm) a trim vertex may sit off the patch and still be
	/// projected onto it — the file's asserted uncertainty times the mode's
	/// factor ([`Importer::snap_allowance`]).
	allow: f64,
	/// Distance (mm) beyond which an accepted vertex is REPORTED as a snap: the
	/// file's own uncertainty — a vertex within it is what the producer asserted,
	/// not a repair.
	report_above: f64,
}

impl NurbsPatch {
	pub(crate) fn new(surf: NurbsSurface, id: u32, allow: f64, report_above: f64) -> Self {
		let grid = patch_seed_grid(&surf);
		let (closed_u, closed_v) = (patch_closed(&surf, true), patch_closed(&surf, false));
		NurbsPatch { surf, grid, id, closed_u, closed_v, allow, report_above }
	}

	/// Surface point at normalised `[0,1]²` coordinates (no wrapping).
	fn at(&self, uv: DVec2) -> DVec3 {
		let ((u_lo, u_hi), (v_lo, v_hi)) = self.surf.domain();
		self.surf.point_at(u_lo + (u_hi - u_lo) * uv.x, v_lo + (v_hi - v_lo) * uv.y)
	}

	fn domain_uv(&self, uv: DVec2) -> (f64, f64) {
		let ((u_lo, u_hi), (v_lo, v_hi)) = self.surf.domain();
		(u_lo + (u_hi - u_lo) * wrap01(uv.x, self.closed_u), v_lo + (v_hi - v_lo) * wrap01(uv.y, self.closed_v))
	}
}

impl ParamPatch for NurbsPatch {
	fn locate(&self, p: DVec3) -> Result<Located, String> {
		let scale = 1.0 + p.length();
		let strict = PATCH_PROJECT_TOL * scale;
		// The projection tolerance is relative (× `1 + |p|`); the uncertainty
		// allowance is absolute — take the looser of the two.
		let tol_rel = PATCH_PROJECT_TOL.max(self.allow / scale);
		let uv = match self.surf.project(&self.grid, p, tol_rel) {
			Some(uv) => uv,
			None => {
				// Retry from a denser seed grid: Newton from the six nearest coarse
				// seeds can miss a vertex on a tightly curled patch.
				let dense = self.surf.projection_seeds(3 * PATCH_SEED_GRID);
				self.surf.project(&dense, p, tol_rel).ok_or_else(|| {
					format!(
						"trim vertex ({:.4}, {:.4}, {:.4}) does not lie on B-spline patch #{} (allowance {:.3e} mm)",
						p.x,
						p.y,
						p.z,
						self.id,
						self.allow.max(strict)
					)
				})?
			}
		};
		let d = (self.at(uv) - p).length();
		Ok(Located { uv, u_defined: true, snapped: (d > strict.max(self.report_above)).then_some(d) })
	}
	fn point(&self, uv: DVec2) -> DVec3 {
		let (u, v) = self.domain_uv(uv);
		self.surf.point_at(u, v)
	}
	fn normal(&self, uv: DVec2) -> DVec3 {
		let (u, v) = self.domain_uv(uv);
		self.surf.normal_at(u, v)
	}
	fn closed(&self) -> (bool, bool) {
		(self.closed_u, self.closed_v)
	}
	fn pole_v(&self, _north: bool) -> Option<f64> {
		None
	}
	fn sag_tol(&self, boundary: &[DVec3]) -> f64 {
		PATCH_SAG_TOL * (1.0 + boundary.iter().map(|p| p.length()).fold(0.0_f64, f64::max))
	}
	fn chart_scale(&self) -> DVec2 {
		DVec2::ONE
	}
	fn facet_surface(&self, centroid: DVec3, normal: DVec3) -> Surface {
		// The analytic [`Surface`] enum has no freeform variant: a triangle IS its
		// plane, so each chord facet carries its own exact plane tag.
		Surface::Plane { origin: centroid, normal }
	}
	fn nurbs(&self) -> Option<&NurbsSurface> {
		Some(&self.surf)
	}
	fn label(&self) -> String {
		format!("B-spline patch #{}", self.id)
	}
}

/// The quadric family of an [`AnalyticPatch`].
#[derive(Clone, Copy)]
enum AnalyticKind {
	Cylinder { radius: f64 },
	Cone { half_angle: f64 },
	Sphere { radius: f64 },
	Torus { major: f64, minor: f64 },
}

/// A trimmed analytic quadric face as a [`ParamPatch`] in its natural periodic
/// chart, normalised so one period is `1`:
///
/// - **cylinder / cone**: `u` = angle about the axis / 2π (closed), `v` = axial
///   distance from the origin/apex over `2π·r_char` (open; the apex has no `u`);
/// - **sphere**: `u` = longitude about the placement axis / 2π (closed), `v` =
///   latitude / 2π ∈ [−¼, ¼] (open; the poles have no `u`);
/// - **torus**: `u` = azimuth / 2π, `v` = tube angle / 2π (both closed).
///
/// `r_char` (the radius; the boundary's largest radius on a cone; `R + r` on a
/// torus) scales the open direction so the chart is near-isometric and sets the
/// refinement's chordal tolerance to the imported-conic contract.
pub(super) struct AnalyticPatch {
	surface: Surface,
	kind: AnalyticKind,
	origin: DVec3,
	axis: DVec3,
	e1: DVec3,
	e2: DVec3,
	v_scale: f64,
	r_char: f64,
	/// Distance (mm) beyond which a boundary vertex off the surface is reported
	/// (the file's own uncertainty); it is kept verbatim either way.
	report_above: f64,
}

impl AnalyticPatch {
	/// `None` for a degenerate surface (zero axis or radius).
	pub(crate) fn new(surface: &Surface, axis: DVec3, boundary: &[DVec3], report_above: f64) -> Option<Self> {
		use std::f64::consts::TAU;
		let axis = axis.normalize_or_zero();
		if axis.length_squared() < 0.5 {
			return None;
		}
		let (e1, e2) = perp_basis(axis);
		let finite_pos = |r: f64| r.is_finite() && r > 0.0;
		let (kind, origin, r_char) = match *surface {
			Surface::Cylinder { origin, radius, .. } => {
				if !finite_pos(radius) {
					return None;
				}
				(AnalyticKind::Cylinder { radius }, origin, radius)
			}
			Surface::Cone { apex, half_angle, .. } => {
				if !(half_angle > 0.0 && half_angle < std::f64::consts::FRAC_PI_2) {
					return None;
				}
				let r = boundary
					.iter()
					.map(|&p| {
						let d = p - apex;
						(d - axis * d.dot(axis)).length()
					})
					.fold(0.0_f64, f64::max);
				if !finite_pos(r) {
					return None;
				}
				(AnalyticKind::Cone { half_angle }, apex, r)
			}
			Surface::Sphere { center, radius } => {
				if !finite_pos(radius) {
					return None;
				}
				(AnalyticKind::Sphere { radius }, center, radius)
			}
			Surface::Torus { center, major, minor, .. } => {
				if !(finite_pos(major) && finite_pos(minor)) {
					return None;
				}
				(AnalyticKind::Torus { major, minor }, center, major + minor)
			}
			Surface::Plane { .. } => return None,
		};
		Some(AnalyticPatch { surface: *surface, kind, origin, axis, e1, e2, v_scale: TAU * r_char, r_char, report_above })
	}
}

impl ParamPatch for AnalyticPatch {
	fn locate(&self, p: DVec3) -> Result<Located, String> {
		use std::f64::consts::TAU;
		let d = p - self.origin;
		let h = d.dot(self.axis);
		let radial = d - self.axis * h;
		let rho = radial.length();
		let u_defined = rho > 1e-9 * (1.0 + d.length());
		let u = if u_defined { radial.dot(self.e2).atan2(radial.dot(self.e1)) / TAU } else { 0.0 };
		let v = match self.kind {
			AnalyticKind::Cylinder { .. } | AnalyticKind::Cone { .. } => h / self.v_scale,
			AnalyticKind::Sphere { .. } => h.atan2(rho) / TAU,
			AnalyticKind::Torus { major, .. } => h.atan2(rho - major) / TAU,
		};
		// A boundary vertex off its analytic surface is kept verbatim (the weld
		// with the neighbouring faces needs the exact position) and reported when
		// it exceeds the allowance — the chart still locates it.
		let off = (self.surface.project(p) - p).length();
		let strict = self.report_above.max(1e-9 * (1.0 + p.length()));
		Ok(Located { uv: DVec2::new(u, v), u_defined, snapped: (off > strict).then_some(off) })
	}
	fn point(&self, uv: DVec2) -> DVec3 {
		use std::f64::consts::TAU;
		let (_, closed_v) = self.closed();
		let phi = wrap01(uv.x, true) * TAU;
		let v = wrap01(uv.y, closed_v);
		let dir = self.e1 * phi.cos() + self.e2 * phi.sin();
		match self.kind {
			AnalyticKind::Cylinder { radius } => self.origin + self.axis * (v * self.v_scale) + dir * radius,
			AnalyticKind::Cone { half_angle } => {
				let h = v * self.v_scale;
				self.origin + self.axis * h + dir * (h * half_angle.tan())
			}
			AnalyticKind::Sphere { radius } => {
				let lat = v * TAU;
				self.origin + (dir * lat.cos() + self.axis * lat.sin()) * radius
			}
			AnalyticKind::Torus { major, minor } => {
				let psi = v * TAU;
				self.origin + dir * (major + minor * psi.cos()) + self.axis * (minor * psi.sin())
			}
		}
	}
	fn normal(&self, uv: DVec2) -> DVec3 {
		self.surface.normal_at(self.point(uv))
	}
	fn closed(&self) -> (bool, bool) {
		(true, matches!(self.kind, AnalyticKind::Torus { .. }))
	}
	fn pole_v(&self, north: bool) -> Option<f64> {
		match self.kind {
			AnalyticKind::Sphere { .. } => Some(if north { 0.25 } else { -0.25 }),
			_ => None,
		}
	}
	fn sag_tol(&self, _boundary: &[DVec3]) -> f64 {
		// The imported-conic fidelity contract: a 48-segment ring's chord sagitta.
		((1.0 - (std::f64::consts::PI / FULL_TURN_SEGMENTS as f64).cos()) * self.r_char).max(1e-9)
	}
	fn chart_scale(&self) -> DVec2 {
		use std::f64::consts::TAU;
		match self.kind {
			AnalyticKind::Torus { major, minor } => DVec2::new(TAU * major, TAU * minor),
			_ => DVec2::new(TAU * self.r_char, self.v_scale),
		}
	}
	fn facet_surface(&self, _centroid: DVec3, _normal: DVec3) -> Surface {
		self.surface
	}
	fn nurbs(&self) -> Option<&NurbsSurface> {
		None
	}
	fn label(&self) -> String {
		match self.kind {
			AnalyticKind::Cylinder { radius } => format!("cylinder r={radius}"),
			AnalyticKind::Cone { half_angle } => format!("cone half-angle={half_angle}"),
			AnalyticKind::Sphere { radius } => format!("sphere r={radius}"),
			AnalyticKind::Torus { major, minor } => format!("torus R={major} r={minor}"),
		}
	}
}

/// Unwrap one trim ring's normalised `uv` into the universal cover of a closed
/// patch: every step is taken the short way around (within half a period — the
/// same half-turn convention as the analytic periodic wall), so a chord crossing
/// the parameter seam continues into the neighbouring period instead of jumping
/// back across the domain, and a seam edge's two traversals land one period
/// apart (the duplicated seam parameters; their identical 3-D positions weld on
/// interning). Returns the loop's net winding in whole periods per direction —
/// `0` for every disk-bounding loop; the closing chord back to the first vertex
/// is also a short-way step, so the winding is `round(last − first)` exactly.
///
/// The ring may contain vertices WITHOUT a defined `u` (a sphere pole, a cone
/// apex): the defined vertices are chained as above, then each undefined vertex takes the `u`
/// interpolated between its flanking defined neighbours (continuing across the
/// loop start, like the analytic wall splitter's apex rule). A pole is where the
/// loop may legitimately jump a whole period (both seam traversals meet there),
/// which the short-way chain cannot see: when the ring closes with a net
/// winding although it contains such a vertex, the part after the first one is
/// shifted back by that winding so the loop bounds a disk in the cover.
/// `defined` is indexed by VERTEX index (parallel to `uv`).
pub(super) fn unwrap_ring_defined(uv: &mut [DVec2], defined: &[bool], ring: &[usize], closed_u: bool, closed_v: bool) -> (i64, i64) {
	let wrap = |d: f64| d - d.round();
	let n = ring.len();
	if closed_v {
		for k in 1..n {
			let (prev, cur) = (uv[ring[k - 1]], uv[ring[k]]);
			uv[ring[k]].y = prev.y + wrap(cur.y - prev.y);
		}
	}
	let mut winding_u = 0i64;
	if closed_u {
		let def: Vec<usize> = (0..n).filter(|&k| defined[ring[k]]).collect();
		if !def.is_empty() {
			for w in def.windows(2) {
				let (a, b) = (ring[w[0]], ring[w[1]]);
				uv[b].x = uv[a].x + wrap(uv[b].x - uv[a].x);
			}
			let (f, l) = (ring[def[0]], ring[def[def.len() - 1]]);
			winding_u = (uv[l].x - uv[f].x).round() as i64;
			if def.len() < n && winding_u != 0 {
				// Undo the net winding at the first undefined vertex.
				let first_undef = (0..n).find(|&k| !defined[ring[k]]).expect("def.len() < n");
				for &k in ring.iter().skip(first_undef + 1) {
					uv[k].x -= winding_u as f64;
				}
				winding_u = 0;
			}
			for k in 0..n {
				if defined[ring[k]] {
					continue;
				}
				let (mut a, mut da) = ((k + n - 1) % n, 1usize);
				while !defined[ring[a]] {
					a = (a + n - 1) % n;
					da += 1;
				}
				let (mut b, mut db) = ((k + 1) % n, 1usize);
				while !defined[ring[b]] {
					b = (b + 1) % n;
					db += 1;
				}
				let ua = uv[ring[a]].x;
				// The neighbour across the loop start continues the chain by a
				// short-way step, not by its own unwrapped value.
				let ub = if b > k { uv[ring[b]].x } else { ua + wrap(uv[ring[b]].x - ua) };
				uv[ring[k]].x = ua + (ub - ua) * da as f64 / (da + db) as f64;
			}
		}
	}
	let winding_v = if closed_v { (uv[ring[n - 1]].y - uv[ring[0]].y).round() as i64 } else { 0 };
	(winding_u, winding_v)
}

/// Which side of a one-rim cap's rim the face region lies on: `true` toward
/// increasing `v` (the north pole). From the rim's first step `d`, the face's
/// material normal `n` (`same_sense` selects the surface's own orientation or
/// its opposite) and the direction `t` of increasing `v`: the region is to the
/// LEFT of the walk, `(n × d) · t > 0`.
pub(super) fn cap_faces_north(patch: &dyn ParamPatch, uv: &[DVec2], pts3: &[DVec3], rim: &[usize], same_sense: bool) -> bool {
	let n_rim = rim.len();
	for k in 0..n_rim {
		let (i, j) = (rim[k], rim[(k + 1) % n_rim]);
		let d = pts3[j] - pts3[i];
		if d.length_squared() < 1e-24 {
			continue;
		}
		let n = patch.normal(uv[i]) * if same_sense { 1.0 } else { -1.0 };
		let t = (patch.point(uv[i] + DVec2::new(0.0, 1e-4)) - patch.point(uv[i])).normalize_or_zero();
		let s = n.cross(d).dot(t);
		if s.abs() > 1e-12 {
			return s > 0.0;
		}
	}
	true
}

/// Close a one-rim cap in the universal cover (mirroring [`bridge_band_rings`]):
/// the rim is extended with a duplicate of its first vertex one period along its
/// travel, then joined to the pole row — two copies of the pole point at the two
/// ends of the period, both the same 3-D position. The two meridian chords
/// (rim start → pole) are one period apart in the cover but bit-identical in
/// 3-D — a synthetic seam whose copies intern to the same vertices and pair as
/// twins — and the pole segment is zero-length in 3-D (its facets degenerate and
/// are dropped on emission, leaving a clean fan around the pole vertex).
pub(super) fn close_cap_ring(uv: &mut Vec<DVec2>, pts3: &mut Vec<DVec3>, patch: &dyn ParamPatch, rim: &[usize], v_pole: f64) -> Vec<usize> {
	let dir = (uv[*rim.last().expect("rims are non-empty")].x - uv[rim[0]].x).signum();
	let u0 = uv[rim[0]].x;
	let a_dup = uv.len();
	uv.push(DVec2::new(u0 + dir, uv[rim[0]].y));
	pts3.push(pts3[rim[0]]);
	let pole = patch.point(DVec2::new(u0, v_pole));
	let p1 = uv.len();
	uv.push(DVec2::new(u0 + dir, v_pole));
	pts3.push(pole);
	let p2 = uv.len();
	uv.push(DVec2::new(u0, v_pole));
	pts3.push(pole);
	let mut merged = rim.to_vec();
	merged.extend([a_dup, p1, p2]);
	merged
}

/// Hard cap on facets per analytic patch face under the batched refinement (a
/// 48-pitch grid over the largest quadric face a vendor part carries is a few
/// thousand; the cap is a loud refusal on a pathological chart).
pub(super) const ANALYTIC_FACET_BUDGET: usize = 60_000;

/// Shortest interior edge (mm, in the metric-scaled chart) the batched
/// refinement still splits — `2π·r_char / 2048` is ~0.18° of arc, far under
/// any chordal target, expressed per patch through [`ParamPatch::chart_scale`].
pub(super) const ANALYTIC_MIN_EDGE_FRACTION: f64 = 1.0 / 2048.0;

/// Minimum facet pitch (fraction of the larger chart scale) behind the batched
/// refinement's **area floor** — the analytic counterpart of
/// [`PATCH_MIN_PITCH`]: an edge all of whose owners have (twice-)area at or
/// below `pitch²/2` is never split. This is the termination device against an
/// unsplittable trim chord (a 90° arc imported as ONE chord, the importer's
/// own-export contract): the strip hugging it can only be "refined" by driving
/// its apex onto the chord — infinitely many splits with the edge lengths never
/// shrinking (measured: a corner ball's octant cycled for 40 rounds). Pinned at
/// the floor, the strip keeps a bounded residual sag.
pub(super) const ANALYTIC_MIN_PITCH: f64 = 1.0 / 256.0;
