// Copyright (c) LMCAD. Licensed under the MIT License.

//! The shared trimmed-face read path: locate every trim vertex on the patch,
//! unwrap the rings through periodic seams, triangulate in parameter space,
//! refine the interior on the exact surface, and emit the facets.

use kernel_core::math::{DVec2, DVec3};

use crate::geom::Surface;
use crate::nurbs::FreeformFace;
use crate::topo::FaceLoops;

use super::edges::pos_key;
use super::face::FaceAccum;
use super::importer::Importer;
use super::patch::{cap_faces_north, close_cap_ring, unwrap_ring_defined, AnalyticPatch, NurbsPatch, ParamPatch};
use super::patch_tess::{
	bridge_band_rings, edge_key, refine_param_facets, refine_param_facets_batched, sample_synthetic_seams, triangulate_trim_rings,
};
use super::triangulate::{triangulate_earclip, triangulate_monotone};
use super::StepError;

/// Tessellate one trimmed face **on its exact parameter patch** — the shared
/// read path of trimmed B-spline faces and of analytic quadric faces the
/// seam-aware splitters cannot read (holes on a curved face, a corner ball
/// bounded by three arcs, rims off the grid phase, a half-torus wall):
///
/// 1. every trim-loop vertex is located in normalised parameter space
///    ([`ParamPatch::locate`] — a vertex off the patch beyond the file's
///    uncertainty allowance is a loud refusal; one accepted only under the
///    allowance is a reported snap);
/// 2. on a CLOSED (periodic) direction the loops are unwrapped into the
///    universal cover ([`unwrap_ring_defined`]): seam-crossing chords continue
///    into the neighbouring period, a slit seam's two traversals land one period
///    apart, a two-rim band is bridged into one ring ([`bridge_band_rings`]),
///    a one-rim cap is closed through the pole row ([`close_cap_ring`]), and hole
///    rings are translated by whole periods onto the outer ring's window;
/// 3. the rings are triangulated in parameter space (monotone sweep for a single
///    ring, hole-bridging ear clip via [`triangulate_trim_rings`] otherwise);
/// 4. the interior is refined to the patch's chordal tolerance
///    ([`refine_param_facets`]) with every new vertex EVALUATED on the exact
///    surface. Trim-loop chords are never subdivided, so the boundary stays
///    bit-identical with the neighbouring faces' edges and the weld is watertight.
///
/// Facets carry [`ParamPatch::facet_surface`]'s tag: the analytic surface for a
/// quadric, a per-facet exact plane for a B-spline patch (whose NURBS identity is
/// preserved in the [`FreeformFace`] sidecar instead). A facet whose three
/// vertices intern to fewer than three distinct positions (a pole-row or
/// synthetic-seam sliver) is zero-area in 3-D and dropped — its two real edges
/// are each other's twins, so the surface stays closed.
pub(super) fn add_patch_face(
	fid: u32,
	patch: &dyn ParamPatch,
	outer_pts: &[DVec3],
	inner_loops: &[Vec<DVec3>],
	face_same_sense: bool,
	acc: &mut FaceAccum,
) -> Result<(), StepError> {
	let unsupported = |m: String| StepError::Unsupported(format!("ADVANCED_FACE #{fid}: {m}"));
	let mut pts3: Vec<DVec3> = Vec::new();
	let mut uv: Vec<DVec2> = Vec::new();
	let mut u_defined: Vec<bool> = Vec::new();
	let mut rings: Vec<Vec<usize>> = Vec::new();
	for lp in std::iter::once(outer_pts).chain(inner_loops.iter().map(Vec::as_slice)) {
		let base = pts3.len();
		for &p in lp {
			let loc = patch.locate(p).map_err(&unsupported)?;
			if let Some(d) = loc.snapped {
				acc.repairs.push((
					fid,
					format!(
						"trim vertex ({:.4}, {:.4}, {:.4}) sat {:.3e} mm off {} and was projected onto it",
						p.x,
						p.y,
						p.z,
						d,
						patch.label()
					),
				));
			}
			pts3.push(p);
			uv.push(loc.uv);
			u_defined.push(loc.u_defined);
		}
		rings.push((base..pts3.len()).collect());
	}
	let (closed_u, closed_v) = patch.closed();
	if closed_u || closed_v {
		let windings: Vec<(i64, i64)> = rings.iter().map(|r| unwrap_ring_defined(&mut uv, &u_defined, r, closed_u, closed_v)).collect();
		let wound: Vec<usize> = (0..rings.len()).filter(|&i| windings[i] != (0, 0)).collect();
		let unit = |w: (i64, i64)| (w.0.abs() <= 1 && w.1.abs() <= 1) && (w.0 == 0) != (w.1 == 0);
		let refuse = || {
			unsupported(format!(
				"trimming loops wind {windings:?} periods around closed {} — only seam-crossing disk loops, a one-rim cap and a band between two opposite full-period rims are importable",
				patch.label()
			))
		};
		match wound.len() {
			// Every loop bounds a disk in the cover (seam-crossing/slit loops).
			0 => {}
			// A one-rim CAP on a surface with poles: the outer rim winds the closed
			// direction once and the region runs from it to the pole on the loop's
			// material side — closed like a band whose second rim is the pole row.
			1 if wound[0] == 0 && windings[0].1 == 0 && windings[0].0.abs() == 1 && patch.pole_v(true).is_some() => {
				let north = cap_faces_north(patch, &uv, &pts3, &rings[0], face_same_sense);
				let v_pole = patch.pole_v(north).expect("pole_v(true) is Some");
				let rim = rings.remove(0);
				let merged = close_cap_ring(&mut uv, &mut pts3, patch, &rim, v_pole);
				// Chord A: rim duplicate → pole copy (position rim.len()); B: pole → rim start.
				let merged =
					if patch.nurbs().is_none() { sample_synthetic_seams(&mut uv, &mut pts3, &merged, rim.len(), patch) } else { merged };
				rings.insert(0, merged);
			}
			// A full-period band: exactly two rims wind ONE closed direction once,
			// in opposite senses, the outer bound being one of them (the untrimmed
			// closed patch, e.g. a NURBS tube wall bounded only by its two rims, a
			// torus band between two rims off the grid phase).
			2 if wound[0] == 0 => {
				let (wa, wb) = (windings[wound[0]], windings[wound[1]]);
				let in_u = wa.0 != 0;
				if !(unit(wa) && unit(wb) && wa.0 + wb.0 == 0 && wa.1 + wb.1 == 0) {
					return Err(refuse());
				}
				let rim_b = rings.remove(wound[1]);
				let rim_a = rings.remove(0);
				let merged = bridge_band_rings(&mut uv, &mut pts3, &rim_a, &rim_b, in_u);
				// Chord A: rim-a duplicate → rim b start (position rim_a.len()); B: rim-b duplicate → rim a start.
				let merged =
					if patch.nurbs().is_none() { sample_synthetic_seams(&mut uv, &mut pts3, &merged, rim_a.len(), patch) } else { merged };
				rings.insert(0, merged);
			}
			_ => return Err(refuse()),
		}
		// Each ring was unwrapped from its own first vertex, so hole rings may sit
		// whole periods away from the outer ring's cover window: translate them onto
		// it (mean-coordinate difference, rounded to whole periods). A hole that
		// still falls outside the outer loop fails the ear clip loudly below.
		let mean = |ring: &[usize], uv: &[DVec2]| ring.iter().map(|&i| uv[i]).fold(DVec2::ZERO, |a, q| a + q) / ring.len() as f64;
		let outer_mean = mean(&rings[0], &uv);
		for ring in rings.iter().skip(1) {
			let d = outer_mean - mean(ring, &uv);
			let shift = DVec2::new(if closed_u { d.x.round() } else { 0.0 }, if closed_v { d.y.round() } else { 0.0 });
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
		triangulate_monotone(&ring_uv)
			.or_else(|_| triangulate_earclip(&ring_uv))
			.map(|ts| ts.into_iter().map(|t| [ring[t[0]], ring[t[1]], ring[t[2]]]).collect::<Vec<_>>())
			.or_else(|_| {
				// Swap the sweep axis: a v-monotone loop is u-monotone in the
				// transposed plane; transposition mirrors the winding, so swap it back.
				let swapped: Vec<DVec2> = ring_uv.iter().map(|q| DVec2::new(q.y, q.x)).collect();
				triangulate_monotone(&swapped)
					.or_else(|_| triangulate_earclip(&swapped))
					.map(|ts| ts.into_iter().map(|t| [ring[t[0]], ring[t[2]], ring[t[1]]]).collect::<Vec<_>>())
			})
	});
	let mut tris = match monotone {
		Some(Ok(ts)) => ts,
		_ => triangulate_trim_rings(&uv, &rings).map_err(unsupported)?,
	};
	// Trim-loop segments are the watertight boundary: never split.
	let boundary: std::collections::HashSet<(usize, usize)> = rings
		.iter()
		.flat_map(|r| {
			let n = r.len();
			(0..n).map(move |i| edge_key(r[i], r[(i + 1) % n]))
		})
		.collect();
	// Interior refinement to the patch's chordal tolerance; cover coordinates of
	// a closed direction wrap back into the fundamental domain for evaluation.
	let eval = |q: DVec2| patch.point(q);
	let sag = patch.sag_tol(&pts3);
	// Boundary handles keep their exact input positions (the weld); interior
	// handles are evaluated on the exact patch as refinement creates them.
	let mut pos3 = pts3.clone();
	// A B-spline patch keeps the single-split Rivara refinement (its sliver control
	// on freeform patches, and the byte-identical results of every existing
	// round trip); an analytic patch — smooth, near-isometric chart, faces of
	// thousands of facets — takes the linear batched refinement.
	let refined = if patch.nurbs().is_some() {
		refine_param_facets(&mut uv, &mut pos3, &mut tris, &boundary, eval, sag)
	} else {
		refine_param_facets_batched(&mut uv, &mut pos3, &mut tris, &boundary, eval, sag, patch.chart_scale())
	};
	refined.map_err(|m| StepError::Unsupported(format!("ADVANCED_FACE #{fid}: {m}")))?;
	let mut emitted = 0usize;
	for t in &tris {
		let (pa, pb, pc) = (pos3[t[0]], pos3[t[1]], pos3[t[2]]);
		if pos_key(pa) == pos_key(pb) || pos_key(pb) == pos_key(pc) || pos_key(pc) == pos_key(pa) {
			continue; // a pole-row / synthetic-seam sliver: zero 3-D area, twins outside
		}
		let (a, b, c) = (acc.intern(pa), acc.intern(pb), acc.intern(pc));
		let centroid = (pa + pb + pc) / 3.0;
		let mut normal = (pb - pa).cross(pc - pa).normalize_or_zero();
		if normal.length_squared() < 0.5 {
			normal = patch.normal((uv[t[0]] + uv[t[1]] + uv[t[2]]) / 3.0);
		}
		if normal.length_squared() < 0.5 {
			return Err(StepError::Topology(format!(
				"ADVANCED_FACE #{fid}: a patch facet has no usable normal (degenerate surface region)"
			)));
		}
		acc.faces.push(FaceLoops { loops: vec![vec![a, b, c]], surface: patch.facet_surface(centroid, normal) });
		emitted += 1;
	}
	if emitted == 0 {
		return Err(StepError::Topology(format!("ADVANCED_FACE #{fid}: every patch facet degenerated")));
	}
	// Preserve a B-spline patch's NURBS identity alongside its chord facets (the
	// analytic [`Surface`] enum has no freeform variant): the exact rational surface
	// plus the verbatim trim rings — the sidecar [`import_step_freeform`] returns and
	// [`crate::step_export::export_step_freeform`] writes back out as a true
	// `B_SPLINE_SURFACE_WITH_KNOTS` face.
	if let Some(surf) = patch.nurbs() {
		acc.freeform.push(FreeformFace {
			surface: surf.clone(),
			rings: std::iter::once(outer_pts.to_vec()).chain(inner_loops.iter().cloned()).collect(),
		});
	}
	Ok(())
}

/// Import one trimmed `B_SPLINE_SURFACE_WITH_KNOTS` face through
/// [`add_patch_face`] on a [`NurbsPatch`].
pub(super) fn add_bspline_face(
	imp: &Importer,
	fid: u32,
	surface_ref: u32,
	outer_pts: &[DVec3],
	inner_loops: &[Vec<DVec3>],
	face_same_sense: bool,
	acc: &mut FaceAccum,
) -> Result<(), StepError> {
	let surf = imp.bspline_surface(surface_ref)?;
	let ((u_lo, u_hi), (v_lo, v_hi)) = surf.domain();
	if !(u_hi > u_lo && v_lo < v_hi) {
		return Err(StepError::Unsupported(format!(
			"ADVANCED_FACE #{fid}: B-spline patch #{surface_ref} has a degenerate parameter domain"
		)));
	}
	let patch = NurbsPatch::new(surf, surface_ref, imp.snap_allowance(), imp.uncertainty);
	add_patch_face(fid, &patch, outer_pts, inner_loops, face_same_sense, acc)
}

/// Import one trimmed analytic quadric face through [`add_patch_face`] on an
/// [`AnalyticPatch`] — the read path for holes on curved faces and for the
/// periodic regions the seam-aware splitters refuse.
#[allow(clippy::too_many_arguments)]
pub(super) fn add_analytic_patch_face(
	imp: &Importer,
	fid: u32,
	surface: &Surface,
	axis: DVec3,
	outer_pts: &[DVec3],
	inner_loops: &[Vec<DVec3>],
	face_same_sense: bool,
	acc: &mut FaceAccum,
) -> Result<(), StepError> {
	let Some(patch) = AnalyticPatch::new(surface, axis, outer_pts, imp.uncertainty) else {
		return Err(StepError::Topology(format!("ADVANCED_FACE #{fid}: degenerate analytic surface (zero axis or radius)")));
	};
	add_patch_face(fid, &patch, outer_pts, inner_loops, face_same_sense, acc)
}
