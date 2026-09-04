// Copyright (c) LMCAD. Licensed under the MIT License.

//! Steps 3 and 4 of the boolean pipeline: classify each co-refined fragment
//! against the *other* operand with a robust ray cast, then select and orient
//! the fragments the boolean rule keeps.

use kernel_core::math::DVec3;

use crate::tol::EPS;

use super::arrange::point_in_tri_3d;
use super::par::{stage_flat_map, CLASSIFY_CHUNK_WORK, CLASSIFY_WORK_CUTOFF, PAR_CHUNK};
use super::{Op, Tri};

// --- Step 3 + 4: classification and selection --------------------------------

/// Where a fragment's centroid lies relative to the *other* solid.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Side {
	Inside,
	Outside,
	/// The fragment is coplanar with (and overlapping) a face of the other solid.
	/// `aligned` is true when the two surface normals point the same way.
	On {
		aligned: bool,
	},
}

/// Classify each fragment of one operand against the `other` solid's triangles
/// and return the kept (and possibly flipped) fragments, in fragment order.
///
/// Fragments coplanar with a face of the other solid (shared/coincident facets)
/// are resolved by normal agreement so they appear in the output exactly once:
/// to avoid double-counting a shared face, only the A-side keeps coincident
/// faces; the B-side drops them.
///
/// Pure per-FRAGMENT map (parallelism-safe, see the module's parallelism
/// section): each fragment's verdict reads only that fragment and the read-only
/// `other` triangle list — the ray-cast scans `other` in slice order with fixed
/// retry directions, no shared accumulator anywhere.
pub(super) fn classify_select(frags: &[Tri], other: &[Tri], op: Op, is_b: bool, workers: usize) -> Vec<Tri> {
	// Work-based scheduling (output-invariant, see CLASSIFY_WORK_CUTOFF): each
	// fragment costs one scan of `other`, so total work — and the chunk length
	// that yields ~CLASSIFY_CHUNK_WORK units per chunk — scales with |other|.
	let work = frags.len().saturating_mul(other.len().max(1));
	let chunk_len = (CLASSIFY_CHUNK_WORK / other.len().max(1)).clamp(1, PAR_CHUNK);
	stage_flat_map(workers, frags, chunk_len, work >= CLASSIFY_WORK_CUTOFF, |chunk| {
		let mut kept: Vec<Tri> = Vec::new();
		for &t in chunk {
			if t.is_degenerate() {
				continue;
			}
			let side = classify_point(t.centroid(), t.normal, other);
			let keep = match side {
				Side::Inside => match op {
					Op::Union => false,
					Op::Intersection => true,
					Op::Difference => is_b, // A inside B removed; B inside A kept (flipped)
				},
				Side::Outside => match op {
					Op::Union => true,
					Op::Intersection => false,
					Op::Difference => !is_b, // A outside B kept; B outside A removed
				},
				Side::On { aligned } => {
					// Coincident faces lie on both solids' boundaries. With this
					// fragment's outward normal `n`, its own material is on the `−n`
					// side; the coincident other face has material on the same side when
					// `aligned`, opposite when not.
					//
					// * Union / intersection: an aligned coincident face has material on
					//   one side and void on the other for the result ⇒ it is a true
					//   boundary ⇒ keep. Opposed faces have material on both sides ⇒
					//   interior ⇒ drop. Both operands emit it; `cancel_coincident`
					//   collapses the duplicate to a single facet.
					// * Difference A−B: where the faces are *aligned*, B's material
					//   coincides with A's and is subtracted away, so A's face vanishes
					//   ⇒ drop. Where *opposed*, B is on the far side and A's face
					//   survives ⇒ keep (A-side only; the flipped B-side is suppressed).
					match (op, is_b) {
						// Aligned coincident faces are a shared boundary that must appear in
						// the result exactly once. Keep the A-side copy and drop the B-side
						// outright (`is_b`), rather than keeping both and trusting
						// `cancel_coincident` to collapse them: when the two faces only
						// partially overlap and were cut by *different* triangulation
						// diagonals, the duplicates are not identical and fail to cancel,
						// leaving a doubled, non-manifold shared face.
						(Op::Union, false) | (Op::Intersection, false) => aligned,
						(Op::Union, true) | (Op::Intersection, true) => false,
						(Op::Difference, false) => !aligned,
						(Op::Difference, true) => false,
					}
				}
			};
			if !keep {
				continue;
			}
			if op == Op::Difference && is_b {
				let mut f = t;
				f.v.swap(1, 2);
				f.normal = -f.normal;
				kept.push(f);
			} else {
				kept.push(t);
			}
		}
		kept
	})
}

/// Three-way classification of point `p` (carrying its fragment `normal`) against
/// the `other` solid. Coplanar coincidence is detected first; otherwise ray
/// casting decides inside/outside.
fn classify_point(p: DVec3, normal: DVec3, other: &[Tri]) -> Side {
	for c in other {
		let nc = c.area_vec();
		let nl = nc.length();
		if nl < EPS {
			continue;
		}
		let ncn = nc / nl;
		// Centroid on this face's plane and inside its triangle?
		if (p - c.v[0]).dot(ncn).abs() <= 1e-7 && point_in_tri_3d(p, c) {
			return Side::On { aligned: normal.dot(ncn) > 0.0 };
		}
	}
	if point_inside(p, other) {
		Side::Inside
	} else {
		Side::Outside
	}
}

/// Robust point-in-polyhedron test by ray casting against `tris`. Counts crossings
/// of a ray from `p` in a pseudo-random direction; odd ⇒ inside. Faces are
/// treated as a closed surface; a small jitter and retry avoids degenerate hits
/// through shared edges/vertices.
fn point_inside(p: DVec3, tris: &[Tri]) -> bool {
	let dirs = [
		DVec3::new(1.0, 1.0, 1.0).normalize(),
		DVec3::new(1.0, 0.0, 1.0).normalize(),
		DVec3::new(0.26726, 0.53452, 0.80178),
		DVec3::new(-0.4082, 0.8165, -0.4082),
		DVec3::new(0.123, -0.567, 0.814).normalize(),
	];
	for &dir in &dirs {
		if let Some(crossings) = ray_crossings(p, dir, tris) {
			return crossings % 2 == 1;
		}
		// `None` ⇒ a near-degenerate hit; retry with another direction.
	}
	false
}

/// Count ray–triangle crossings, or `None` if any hit is numerically ambiguous
/// (ray grazes an edge/vertex/plane), signalling the caller to pick a new ray.
fn ray_crossings(orig: DVec3, dir: DVec3, tris: &[Tri]) -> Option<usize> {
	let mut count = 0usize;
	for t in tris {
		match moller_trumbore(orig, dir, t) {
			Hit::Cross => count += 1,
			Hit::Miss => {}
			Hit::Degenerate => return None,
		}
	}
	Some(count)
}

enum Hit {
	Cross,
	Miss,
	Degenerate,
}

/// Möller–Trumbore ray/triangle test, classifying grazing hits as `Degenerate`
/// so the caller can re-cast and keep parity well-defined.
fn moller_trumbore(orig: DVec3, dir: DVec3, t: &Tri) -> Hit {
	let e1 = t.v[1] - t.v[0];
	let e2 = t.v[2] - t.v[0];
	let pvec = dir.cross(e2);
	let det = e1.dot(pvec);
	if det.abs() < 1e-12 {
		return Hit::Miss; // ray parallel to triangle
	}
	let inv = 1.0 / det;
	let tvec = orig - t.v[0];
	let u = tvec.dot(pvec) * inv;
	let qvec = tvec.cross(e1);
	let v = dir.dot(qvec) * inv;
	let dist = e2.dot(qvec) * inv;
	let edge_tol = 1e-9;
	// Grazing the boundary (barycentric on/near an edge) ⇒ ambiguous parity.
	if u > -edge_tol && u < edge_tol
		|| v > -edge_tol && v < edge_tol
		|| (u + v) > 1.0 - edge_tol && (u + v) < 1.0 + edge_tol
		|| dist.abs() < edge_tol
	{
		// Only ambiguous if the hit is actually within/near the triangle and ahead.
		if u >= -edge_tol && v >= -edge_tol && u + v <= 1.0 + edge_tol && dist > -edge_tol {
			return Hit::Degenerate;
		}
	}
	if !(0.0..=1.0).contains(&u) || v < 0.0 || u + v > 1.0 {
		return Hit::Miss;
	}
	if dist > edge_tol {
		Hit::Cross
	} else {
		Hit::Miss
	}
}
