// Copyright (c) LMCAD. Licensed under the MIT License.

//! The seam-aware splitters for periodic analytic walls and regions: is a curved
//! boundary one flat chord facet, how far does it span, and — when it must be
//! split — the cylindrical/conical/spherical/toroidal frame it is resampled on.

use std::collections::HashMap;

use kernel_core::math::{DVec2, DVec3};

use crate::geom::{perp_basis, Surface};

use super::edges::{complex_part, newell_vector, pos_key, PosKey, FULL_TURN_SEGMENTS, MAX_CHORD_SWEEP};
use super::parse::Entity;
use super::triangulate::{triangulate_earclip, triangulate_monotone};
use super::StepError;

/// Whether an entity is a B-spline surface (plain `B_SPLINE_SURFACE_WITH_KNOTS` or a
/// rational `_COMPLEX` instance carrying that record).
pub(super) fn is_bspline_surface(e: &Entity) -> bool {
	e.name == "B_SPLINE_SURFACE_WITH_KNOTS" || (e.name == "_COMPLEX" && complex_part(&e.args, "B_SPLINE_SURFACE_WITH_KNOTS").is_some())
}

/// Whether a curved-tagged boundary with more than four vertices is a flat CHORD FACET
/// of its surface: planar to tolerance AND spanning at most `MAX_CHORD_SWEEP` of the
/// surface's angular extent. Boolean-recovered bands are such facets (coplanar corners,
/// small sagitta, e.g. a clipped bore wall whose straight cuts added collinear
/// vertices); a real exporter's pole-spanning cap is NOT — its rim is planar but spans
/// the full turn — and must be treated as a curved region instead.
pub(super) fn is_chord_facet(pts: &[DVec3], surface: &Surface) -> bool {
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
pub(super) fn split_periodic_face(pts: &[DVec3], surface: &Surface, fid: u32) -> Result<Vec<[usize; 3]>, StepError> {
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
			return Err(StepError::Topology(format!("ADVANCED_FACE #{fid}: planar face reached the periodic splitter")));
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
		return Err(StepError::Topology(format!("ADVANCED_FACE #{fid}: face boundary collapses onto its surface axis")));
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
		return Err(StepError::Topology(format!("ADVANCED_FACE #{fid}: face boundary has no off-axis points")));
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
	triangulate_monotone(&uv)
		.or_else(|_| triangulate_earclip(&uv))
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
pub(super) fn general_curved_region(pts: &[DVec3], surface: &Surface) -> Option<(Vec<DVec3>, Vec<[usize; 3]>)> {
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
			FrameKind::Torus { major, minor } => self.center + u * (major + minor * level.cos()) + self.axis * (minor * level.sin()),
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
pub(super) fn resample_periodic_region(
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
	Err(StepError::Unsupported(format!("ADVANCED_FACE #{fid}: periodic sphere/torus region not importable — {last_err}")))
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
					return Err(format!("boundary point {:?} is neither on a full ring, a pole, nor a seam traversed twice", pts[i]));
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
	let phi0 = rings.first().map(|r| coords[r.cols[0]].0).unwrap_or_else(|| coords.iter().find(|c| c.2).map(|c| c.0).unwrap_or(0.0));
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
			(nr, np) => return Err(format!("{nr} ring(s) + {np} pole(s) is not a sphere cap, band or full sphere")),
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
	let used_keys: std::collections::HashSet<PosKey> = (0..n_pts).filter(|&i| used[i]).map(|i| pos_key(pts[i])).collect();
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
		let keyed: std::collections::HashSet<(PosKey, PosKey)> = steps.iter().map(|&(i, j)| (pos_key(pts[i]), pos_key(pts[j]))).collect();
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
