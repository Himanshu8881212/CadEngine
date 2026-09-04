// Copyright (c) LMCAD. Licensed under the MIT License.

//! Face building: read one `ADVANCED_FACE`'s bound loops, accumulate vertices and
//! facets into a [`FaceAccum`] (with a checkpoint/rollback so a failed route
//! leaves nothing behind), and route the face to the flat, periodic, patch or
//! flat-repair path.

use std::collections::HashMap;

use kernel_core::math::{DVec2, DVec3};

use crate::geom::{perp_basis, Curve, Surface};
use crate::nurbs::FreeformFace;
use crate::topo::{FaceLoops, Solid, VertexId};

use super::edges::{dedup_ring, last_enum, newell_vector, pos_key};
use super::importer::Importer;
use super::parse::Value;
use super::patch_face::{add_analytic_patch_face, add_bspline_face};
use super::patch_tess::triangulate_trim_rings;
use super::periodic::{general_curved_region, is_bspline_surface, is_chord_facet, resample_periodic_region, split_periodic_face};
use super::triangulate::triangulate_earclip;
use super::StepError;

/// Read one face bound's loop: the tessellated boundary (reversed when the bound's
/// orientation flag says so) and whether it carried any conic segments. Conic segments
/// are recorded by their (direction-independent) endpoint pair *before* any flip, for
/// analytic edge tagging after the solid is built.
pub(crate) fn read_bound_loop(
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
pub(crate) struct FaceAccum {
	pub(crate) positions: Vec<DVec3>,
	pub(crate) index: HashMap<(u64, u64, u64), u32>,
	pub(crate) faces: Vec<FaceLoops>,
	pub(crate) conic_segments: Vec<(DVec3, DVec3, Curve)>,
	pub(crate) edge_cache: HashMap<u32, (Vec<DVec3>, Option<Curve>)>,
	/// The NURBS identity of every trimmed B-spline face reconstructed into `faces`
	/// (which carries chord facets only) — the sidecar [`import_step_freeform`] returns.
	pub(crate) freeform: Vec<FreeformFace>,
	/// `(face id, what was done)` for every silent-in-strict-mode repair the
	/// reconstruction applied (a trim vertex projected onto its patch under the
	/// file's uncertainty allowance) — the tolerant receipt's `repaired` list.
	pub(crate) repairs: Vec<(u32, String)>,
}

/// A snapshot of a [`FaceAccum`]'s lengths, to undo a face's partial output
/// ([`FaceAccum::rollback`]) before retrying it another way. Edge polylines
/// (`edge_cache`) are pure and are kept.
#[derive(Clone, Copy)]
pub(crate) struct AccumCheckpoint {
	positions: usize,
	faces: usize,
	conic_segments: usize,
	freeform: usize,
	repairs: usize,
}

impl FaceAccum {
	pub(crate) fn intern(&mut self, p: DVec3) -> u32 {
		*self.index.entry(pos_key(p)).or_insert_with(|| {
			self.positions.push(p);
			(self.positions.len() - 1) as u32
		})
	}

	pub(crate) fn checkpoint(&self) -> AccumCheckpoint {
		AccumCheckpoint {
			positions: self.positions.len(),
			faces: self.faces.len(),
			conic_segments: self.conic_segments.len(),
			freeform: self.freeform.len(),
			repairs: self.repairs.len(),
		}
	}

	/// Undo everything accumulated since `cp` — including interned positions no
	/// surviving face references (an interned-but-unused position would become an
	/// isolated vertex and corrupt the Euler characteristic).
	pub(crate) fn rollback(&mut self, cp: AccumCheckpoint) {
		if self.positions.len() > cp.positions {
			self.positions.truncate(cp.positions);
			let keep = cp.positions as u32;
			self.index.retain(|_, &mut i| i < keep);
		}
		self.faces.truncate(cp.faces);
		self.conic_segments.truncate(cp.conic_segments);
		self.freeform.truncate(cp.freeform);
		self.repairs.truncate(cp.repairs);
	}

	/// Build the solid from everything accumulated. Faces carry the producer's loop
	/// winding (outward CCW for a well-formed file), so `from_faces_multiloop` pairs
	/// the shared edges into a consistent 2-manifold directly — hole loops included.
	pub(crate) fn finish(self) -> Result<Solid, StepError> {
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

/// The inner message of a [`StepError`] (without the variant prefix), for
/// composing a fallback's reason onto the first attempt's.
fn inner_msg(e: &StepError) -> &str {
	match e {
		StepError::Parse(m) | StepError::Reference(m) | StepError::Unsupported(m) | StepError::Topology(m) => m,
	}
}

/// One `ADVANCED_FACE`'s bounds, read and tessellated: the surface (`None` for a
/// B-spline patch), its entity id, the face's material orientation, the outer
/// loop and the inner (hole) loops — the input every reconstruction route
/// shares ([`read_face_loops`]).
pub(crate) struct FaceRead {
	pub(crate) surface_ref: u32,
	pub(crate) surface: FaceSurface,
	pub(crate) same_sense: bool,
	pub(crate) outer: Vec<DVec3>,
	pub(crate) inner: Vec<Vec<DVec3>>,
}

/// What a face's surface entity resolved to. An unsupported surface type keeps
/// its error here (rather than failing the read outright) so the tolerant
/// importer's flat repair can still consume the face's loops.
pub(crate) enum FaceSurface {
	Analytic(Surface),
	BSpline,
	Unsupported(StepError),
}

/// Read one `ADVANCED_FACE`'s surface and loops. `flip` reverses every loop (an
/// `ORIENTED_CLOSED_SHELL` `.F.` wrapper). `Ok(None)` for a genuinely degenerate
/// (zero-length) planar outer loop, which is skipped as exporters do.
pub(crate) fn read_face_loops(imp: &Importer, fid: u32, flip: bool, acc: &mut FaceAccum) -> Result<Option<FaceRead>, StepError> {
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
	// A B-spline surface face is tessellated on the exact patch; other
	// unsupported surfaces stay loud.
	let surface = match imp.surface(surface_ref) {
		Ok(s) => FaceSurface::Analytic(s),
		Err(StepError::Unsupported(_)) if is_bspline_surface(imp.get(surface_ref)?) => FaceSurface::BSpline,
		Err(e @ StepError::Unsupported(_)) => FaceSurface::Unsupported(e),
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
		return Ok(None); // genuinely degenerate (zero-length) planar loop — skip, as exporters do
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
	Ok(Some(FaceRead { surface_ref, surface, same_sense: face_same_sense, outer: outer_pts, inner: inner_loops }))
}

/// Reconstruct one `ADVANCED_FACE` into `acc` — one input face, or, for a periodic
/// wall / curved region / B-spline patch, a set of facets on its exact surface.
/// `flip` reverses every loop (an `ORIENTED_CLOSED_SHELL` `.F.` wrapper).
///
/// Routing: planar faces keep their loops verbatim; a B-spline face takes the
/// parameter-patch path; a curved analytic face with holes takes it too; a
/// hole-free curved face that is a single chord facet stays one face, otherwise
/// the seam-aware splitters (periodic wall strip, sphere/torus ring grid) read
/// it, and whatever THEY refuse falls back to the parameter-patch path — so the
/// shapes those splitters were measured on keep their exact reconstruction, and
/// the rest (a corner ball bounded by three arcs, rims off the grid phase, a
/// half-torus wall) imports instead of refusing. Any partial output of a failed
/// attempt is rolled back before the next.
pub(crate) fn add_face(imp: &Importer, fid: u32, flip: bool, acc: &mut FaceAccum) -> Result<(), StepError> {
	let Some(face) = read_face_loops(imp, fid, flip, acc)? else { return Ok(()) };
	let FaceRead { surface_ref, surface, same_sense: face_same_sense, outer: outer_pts, inner: inner_loops } = face;

	let surface = match surface {
		FaceSurface::Analytic(s) => s,
		// A trimmed B_SPLINE_SURFACE face: tessellated on the exact patch.
		FaceSurface::BSpline => return add_bspline_face(imp, fid, surface_ref, &outer_pts, &inner_loops, face_same_sense, acc),
		FaceSurface::Unsupported(e) => return Err(e),
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
			Ok(())
		}
		curved => {
			// The unwrap axis: a sphere region unwraps about its placement axis, the
			// other quadrics carry their own.
			let axis = match curved {
				Surface::Cylinder { axis, .. } | Surface::Cone { axis, .. } | Surface::Torus { axis, .. } => axis,
				Surface::Sphere { .. } => imp.surface_axis(surface_ref)?,
				Surface::Plane { .. } => unreachable!("planar faces are handled above"),
			};
			if !inner_loops.is_empty() {
				// Holes on a curved analytic face: the parameter-patch path (the same
				// loop-unwrapping / hole-bridging tessellation trimmed B-spline faces
				// take), facets ON the exact surface carrying its analytic tag.
				return add_analytic_patch_face(imp, fid, &curved, axis, &outer_pts, &inner_loops, face_same_sense, acc);
			}
			if outer_pts.len() <= 4 || is_chord_facet(&outer_pts, &curved) {
				// A native chord facet — ≤4 vertices (this kernel's own cylinder/
				// sphere bands) or a flat, narrow-span polygon on the surface (a
				// boolean-recovered band). Kept as ONE face, matching the
				// tessellator's flat-chord semantics and keeping own-export
				// round-trips exact.
				let idx: Vec<u32> = outer_pts.iter().map(|&p| acc.intern(p)).collect();
				acc.faces.push(FaceLoops { loops: vec![idx], surface: curved });
				return Ok(());
			}
			let cp = acc.checkpoint();
			let first = match curved {
				Surface::Cylinder { .. } | Surface::Cone { .. } => add_periodic_wall(fid, &curved, &outer_pts, acc),
				Surface::Sphere { .. } | Surface::Torus { .. } => add_periodic_region(fid, &curved, axis, face_same_sense, &outer_pts, acc),
				Surface::Plane { .. } => unreachable!("planar faces are handled above"),
			};
			let Err(e) = first else { return Ok(()) };
			acc.rollback(cp);
			let fallback = add_analytic_patch_face(imp, fid, &curved, axis, &outer_pts, &inner_loops, face_same_sense, acc);
			match fallback {
				Ok(()) => Ok(()),
				Err(e2) => {
					acc.rollback(cp);
					Err(StepError::Unsupported(format!("{}; parameter-patch fallback: {}", inner_msg(&e), inner_msg(&e2))))
				}
			}
		}
	}
}

/// A periodic cylinder/cone wall (full-circle rims + a seam edge): the seam-aware
/// unwrap first, then VERIFIED against the parameter chart. The oracle is FLUX,
/// not area: two triangulations of the SAME boundary ring differ in flux by
/// exactly the volume enclosed between them (divergence theorem), so a strip
/// that folds back on itself — what a mesher-jagged merged face's non-monotone
/// unwrap produces — is caught even when its total area looks right. Measured: a
/// recovered implicit cylinder's wall re-imported 37.6% light through the
/// unverified strip. An ordinary chord band (what the exporter coalesces a
/// builder wall into) matches the chart to well under the bar and keeps its
/// exact reconstruction.
fn add_periodic_wall(fid: u32, curved: &Surface, outer_pts: &[DVec3], acc: &mut FaceAccum) -> Result<(), StepError> {
	let charted = general_curved_region(outer_pts, curved);
	let strip = split_periodic_face(outer_pts, curved, fid);
	let strip_ok = match (&strip, &charted) {
		(Ok(tris), Some((extras, ctris))) => {
			let pos = |h: usize| if h < outer_pts.len() { outer_pts[h] } else { extras[h - outer_pts.len()] };
			// Flux about a local anchor (the surface's own origin) keeps the terms
			// at model scale.
			let anchor = match *curved {
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
			// Both windings follow the same input ring, so a healthy strip agrees
			// with the chart to a chord sagitta.
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
			acc.faces.push(FaceLoops { loops: vec![vec![a, b, c]], surface: *curved });
		}
		return Ok(());
	}
	let tris = strip?;
	let idx: Vec<u32> = outer_pts.iter().map(|&p| acc.intern(p)).collect();
	for t in tris {
		let (a, b, c) = (idx[t[0]], idx[t[1]], idx[t[2]]);
		if a == b || b == c || c == a {
			// A facet joining both copies of the seam vertex would be a zero-width
			// sliver; monotone sweep cannot produce one unless the input was
			// degenerate.
			return Err(StepError::Topology(format!("ADVANCED_FACE #{fid}: a split facet degenerated onto the seam")));
		}
		acc.faces.push(FaceLoops { loops: vec![vec![a, b, c]], surface: *curved });
	}
	Ok(())
}

/// A periodic / pole-spanning sphere or torus region: resampled into a ring grid
/// on the exact surface (see [`resample_periodic_region`]). (The cylinder/cone
/// chart of [`general_curved_region`] is NOT tried here: it is not injective on
/// a torus band and read one vendor screw head as overlapping facets.)
fn add_periodic_region(
	fid: u32,
	curved: &Surface,
	axis: DVec3,
	face_same_sense: bool,
	outer_pts: &[DVec3],
	acc: &mut FaceAccum,
) -> Result<(), StepError> {
	let (extras, tris) = resample_periodic_region(outer_pts, curved, axis, face_same_sense, fid)?;
	// Intern lazily, per referenced handle: seam (slit) boundary points are
	// interior to the face and may legitimately go unused — an eagerly interned
	// copy would become an isolated vertex and corrupt the Euler characteristic.
	let pos = |h: usize| if h < outer_pts.len() { outer_pts[h] } else { extras[h - outer_pts.len()] };
	for t in tris {
		let (a, b, c) = (acc.intern(pos(t[0])), acc.intern(pos(t[1])), acc.intern(pos(t[2])));
		if a == b || b == c || c == a {
			return Err(StepError::Topology(format!("ADVANCED_FACE #{fid}: a resampled facet degenerated onto the seam")));
		}
		acc.faces.push(FaceLoops { loops: vec![vec![a, b, c]], surface: *curved });
	}
	Ok(())
}

/// The tolerant importer's last-resort **flat repair** of a face no exact route
/// could read (an unsupported surface type such as `SURFACE_OF_LINEAR_EXTRUSION`,
/// a loop that cannot be charted): its loops are projected onto the outer loop's
/// Newell plane and ear-clipped there (holes bridged), each facet carrying its
/// own exact plane tag. The boundary chords are consumed verbatim, so the shell
/// stays welded and closed; only the face's interior geometry is approximated —
/// which is exactly what the receipt records for it. Refuses (`Err`) when the
/// loop encloses no projected area (a periodic slit loop) or does not bound a
/// simple region there.
pub(crate) fn add_face_flat(imp: &Importer, fid: u32, flip: bool, acc: &mut FaceAccum) -> Result<String, StepError> {
	let cp = acc.checkpoint();
	let Some(face) = read_face_loops(imp, fid, flip, acc)? else {
		return Ok("degenerate zero-length loop skipped".into());
	};
	let surface_name = imp.get(face.surface_ref).map(|e| e.name.clone()).unwrap_or_else(|_| "?".into());
	let nv = newell_vector(&face.outer);
	let scale = face.outer.iter().map(|p| p.length()).fold(0.0_f64, f64::max) + 1.0;
	if nv.length() < 1e-12 * scale * scale {
		acc.rollback(cp);
		return Err(StepError::Unsupported(format!(
			"ADVANCED_FACE #{fid}: the outer loop encloses no projected area (a periodic slit loop) — no flat repair possible"
		)));
	}
	let n = nv.normalize();
	let (e1, e2) = perp_basis(n);
	let mut pts3: Vec<DVec3> = Vec::new();
	let mut uv: Vec<DVec2> = Vec::new();
	let mut rings: Vec<Vec<usize>> = Vec::new();
	for lp in std::iter::once(&face.outer).chain(face.inner.iter()) {
		let base = pts3.len();
		for &p in lp {
			pts3.push(p);
			uv.push(DVec2::new(p.dot(e1), p.dot(e2)));
		}
		rings.push((base..pts3.len()).collect());
	}
	let tris = if rings.len() == 1 {
		triangulate_earclip(&uv).map(|ts| ts.into_iter().map(|t| [rings[0][t[0]], rings[0][t[1]], rings[0][t[2]]]).collect::<Vec<_>>())
	} else {
		triangulate_trim_rings(&uv, &rings)
	};
	let tris = match tris {
		Ok(t) => t,
		Err(m) => {
			acc.rollback(cp);
			return Err(StepError::Unsupported(format!("ADVANCED_FACE #{fid}: flat repair failed — {m}")));
		}
	};
	let mut emitted = 0usize;
	for t in &tris {
		let (pa, pb, pc) = (pts3[t[0]], pts3[t[1]], pts3[t[2]]);
		if pos_key(pa) == pos_key(pb) || pos_key(pb) == pos_key(pc) || pos_key(pc) == pos_key(pa) {
			continue;
		}
		let normal = (pb - pa).cross(pc - pa).normalize_or_zero();
		if normal.length_squared() < 0.5 {
			continue; // collinear sliver in 3-D: zero area, its edges pair outside
		}
		let (a, b, c) = (acc.intern(pa), acc.intern(pb), acc.intern(pc));
		acc.faces.push(FaceLoops { loops: vec![vec![a, b, c]], surface: Surface::Plane { origin: (pa + pb + pc) / 3.0, normal } });
		emitted += 1;
	}
	if emitted == 0 {
		acc.rollback(cp);
		return Err(StepError::Unsupported(format!("ADVANCED_FACE #{fid}: flat repair produced no facet")));
	}
	Ok(format!(
		"{surface_name} face approximated by {emitted} flat facets of its boundary loop{} (interior geometry NOT exact)",
		if face.inner.is_empty() { String::new() } else { format!(" and {} hole loop(s)", face.inner.len()) }
	))
}
