// Copyright (c) LMCAD. Licensed under the MIT License.

//! Parameter-space tessellation of a trimmed patch: the trim-ring triangulation
//! (hole bridging plus ear clip) and the two interior refinement passes — the
//! plain sag-driven one for B-spline patches and the batched Delaunay-flipped one
//! for analytic quadrics.

use std::collections::HashMap;

use kernel_core::math::{DVec2, DVec3};
use kernel_core::orient2d;

use crate::nurbs::NurbsSurface;

use super::edges::FULL_TURN_SEGMENTS;
use super::patch::{ParamPatch, ANALYTIC_FACET_BUDGET, ANALYTIC_MIN_EDGE_FRACTION, ANALYTIC_MIN_PITCH};

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
pub(super) const PATCH_MIN_PITCH: f64 = 1.0 / 256.0;

/// Twice-the-area floor derived from [`PATCH_MIN_PITCH`] (`pitch²/4` of true area).
const AREA_FLOOR2: f64 = PATCH_MIN_PITCH * PATCH_MIN_PITCH / 2.0;

/// The coarse `(normalised uv, position)` seed grid of a patch, evaluated once per
/// face and shared across all of its trim-vertex projections
/// ([`NurbsSurface::projection_seeds`] at the [`PATCH_SEED_GRID`] resolution).
pub(super) fn patch_seed_grid(surf: &NurbsSurface) -> Vec<(DVec2, DVec3)> {
	surf.projection_seeds(PATCH_SEED_GRID)
}

/// Undirected edge key.
pub(super) fn edge_key(a: usize, b: usize) -> (usize, usize) {
	(a.min(b), a.max(b))
}

/// Whether a patch is geometrically closed (periodic) across its `u` domain ends —
/// `S(u_lo, v) = S(u_hi, v)` along the whole seam to relative tolerance. These are
/// the closed patches (NURBS cylinders and friends) whose seam a trim loop may
/// legitimately cross, handled by unwrapping into the universal cover. `in_u =
/// false` checks the `v` direction. The check EVALUATES the surface (not a
/// control-net heuristic), so a wrapped net with mismatched weight rows — whose
/// seam genuinely gapes — does not false-positive.
pub(super) fn patch_closed(surf: &NurbsSurface, in_u: bool) -> bool {
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
pub(super) fn bridge_band_rings(uv: &mut Vec<DVec2>, pts3: &mut Vec<DVec3>, ring_a: &[usize], ring_b: &[usize], in_u: bool) -> Vec<usize> {
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
	let rot = (0..n_b).min_by(|&i, &j| nearest(i).abs().total_cmp(&nearest(j).abs())).expect("rims are non-empty");
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
pub(crate) fn triangulate_trim_rings(uv: &[DVec2], rings: &[Vec<usize>]) -> Result<Vec<[usize; 3]>, String> {
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
		let &h = hole.iter().max_by(|&&i, &&j| uv[i].x.total_cmp(&uv[j].x)).expect("holes are non-empty rings");
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
pub(super) fn refine_param_facets(
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
					*sag_cache.entry((a, b)).or_insert_with(|| (eval((uv[a] + uv[b]) * 0.5) - (pos[a] + pos[b]) * 0.5).length())
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
						&& l > len2(a, b) && live(ek, &adj, tris)
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

/// Lawson edge flips towards the Delaunay triangulation of `tris` in the
/// metric-scaled chart (`uv ∘ scale`): an interior edge whose two owners fail
/// the in-circle test is flipped when the quad is strictly convex. Boundary
/// (trim-loop) edges are never flipped. Windings are preserved (every triangle
/// keeps the input orientation). Returns the number of flips.
fn lawson_flips(
	uv: &[DVec2],
	tris: &mut [[usize; 3]],
	boundary: &std::collections::HashSet<(usize, usize)>,
	scale: DVec2,
	sag: &dyn Fn(usize, usize) -> f64,
	sag_tol: f64,
) -> usize {
	let at = |i: usize| DVec2::new(uv[i].x * scale.x, uv[i].y * scale.y);
	let orient = |a: DVec2, b: DVec2, c: DVec2| orient2d([a.x, a.y], [b.x, b.y], [c.x, c.y]);
	// In-circle of `d` against the CCW triangle (a, b, c): > 0 inside.
	let in_circle = |a: DVec2, b: DVec2, c: DVec2, d: DVec2| -> f64 {
		let (ax, ay) = (a.x - d.x, a.y - d.y);
		let (bx, by) = (b.x - d.x, b.y - d.y);
		let (cx, cy) = (c.x - d.x, c.y - d.y);
		(ax * ax + ay * ay) * (bx * cy - cx * by) - (bx * bx + by * by) * (ax * cy - cx * ay) + (cx * cx + cy * cy) * (ax * by - bx * ay)
	};
	let mut flips = 0usize;
	for _pass in 0..12 {
		// Directed half-edge → owning triangle index.
		let mut owner: HashMap<(usize, usize), usize> = HashMap::with_capacity(tris.len() * 3);
		for (ti, t) in tris.iter().enumerate() {
			for k in 0..3 {
				owner.insert((t[k], t[(k + 1) % 3]), ti);
			}
		}
		let mut flipped_this_pass = 0usize;
		let mut touched = vec![false; tris.len()];
		for ti in 0..tris.len() {
			if touched[ti] {
				continue;
			}
			let t = tris[ti];
			for k in 0..3 {
				let (a, b, c) = (t[k], t[(k + 1) % 3], t[(k + 2) % 3]);
				if boundary.contains(&edge_key(a, b)) {
					continue;
				}
				let Some(&tj) = owner.get(&(b, a)) else { continue };
				if tj == ti || touched[tj] {
					continue;
				}
				let u = tris[tj];
				let Some(kd) = (0..3).find(|&m| u[m] == b && u[(m + 1) % 3] == a) else { continue };
				let d = u[(kd + 2) % 3];
				if d == c {
					continue;
				}
				let (pa, pb, pc, pd) = (at(a), at(b), at(c), at(d));
				// Both triangles must be positively oriented for the test to apply.
				let sign = orient(pa, pb, pc);
				if sign == 0.0 || orient(pb, pa, pd).signum() != sign.signum() {
					continue;
				}
				let inside = if sign > 0.0 { in_circle(pa, pb, pc, pd) } else { -in_circle(pa, pb, pc, pd) };
				if inside <= 0.0 {
					continue;
				}
				// Flip a–b → c–d: (a, d, c) and (d, b, c), both strictly oriented like the input.
				let o1 = orient(pa, pd, pc);
				let o2 = orient(pd, pb, pc);
				if o1.signum() != sign.signum() || o2.signum() != sign.signum() {
					continue; // non-convex quad
				}
				// Never trade for an ABOVE-tolerance diagonal that sags more off the
				// surface than the one it replaces: the chart is not isometric (a
				// sphere chart near its pole), and a flip that is Delaunay there can
				// re-create the very chord the previous round split — the refinement
				// would then never converge. A diagonal already within tolerance
				// needs no split, so it can never cycle and is always allowed (that
				// is what lets the flips remove the zig-zag slivers whose chords
				// over-read a curved face's area).
				let s_new = sag(c, d);
				if s_new > sag_tol && s_new > sag(a, b) {
					continue;
				}
				tris[ti] = [a, d, c];
				tris[tj] = [d, b, c];
				touched[ti] = true;
				touched[tj] = true;
				flipped_this_pass += 1;
				break;
			}
		}
		flips += flipped_this_pass;
		if flipped_this_pass == 0 {
			break;
		}
	}
	flips
}

/// **Batched** conforming chordal refinement of a parameter-space
/// triangulation — the analytic-patch counterpart of [`refine_param_facets`].
/// Where that routine splits ONE edge per round (Rivara's longest-edge walk,
/// chosen for sliver control on freeform patches) and is therefore quadratic in
/// the facet count, this one splits EVERY qualifying interior edge of a round
/// at once — the standard 1→2 / 1→3 / 1→4 patterns per triangle, both owners
/// of an edge sharing its one midpoint, so the mesh never grows a T-junction —
/// and restores triangle quality after each round with Lawson flips in the
/// metric-scaled chart ([`lawson_flips`]): without them the long fan slivers
/// the monotone sweep seeds across a strip survive every split and, being
/// zig-zag chords on a curved surface, over-read its area (Schwarz's lantern —
/// measured +1.2% on a half-torus wall at 20 k facets). An edge qualifies while
/// its straight chord sags more than `sag_tol` from the exact surface at its
/// chart midpoint and it is longer than the [`ANALYTIC_MIN_EDGE_FRACTION`]
/// floor; boundary (trim-loop) edges are never split. Every midpoint is
/// evaluated on the exact surface through `eval`. Facet windings follow the
/// parent's.
pub(super) fn refine_param_facets_batched(
	uv: &mut Vec<DVec2>,
	pos: &mut Vec<DVec3>,
	tris: &mut Vec<[usize; 3]>,
	boundary: &std::collections::HashSet<(usize, usize)>,
	eval: impl Fn(DVec2) -> DVec3,
	sag_tol: f64,
	scale: DVec2,
) -> Result<(), String> {
	let uv_key = |q: DVec2| (q.x.to_bits(), q.y.to_bits());
	let mut by_uv: HashMap<(u64, u64), usize> = uv.iter().enumerate().map(|(i, &q)| (uv_key(q), i)).collect();
	let min_len = ANALYTIC_MIN_EDGE_FRACTION * scale.x.max(scale.y);
	let min_len2 = min_len * min_len;
	let pitch = ANALYTIC_MIN_PITCH * scale.x.max(scale.y);
	let floor2 = pitch * pitch / 2.0;
	// Twice the scaled chart area of a facet.
	let area2 = |uv: &[DVec2], t: &[usize; 3]| {
		let s = |i: usize| DVec2::new(uv[i].x * scale.x, uv[i].y * scale.y);
		(s(t[1]) - s(t[0])).perp_dot(s(t[2]) - s(t[0])).abs()
	};
	let sag_of = |uv: &[DVec2], pos: &[DVec3], a: usize, b: usize| (eval((uv[a] + uv[b]) * 0.5) - (pos[a] + pos[b]) * 0.5).length();
	{
		let s = |a: usize, b: usize| sag_of(uv, pos, a, b);
		lawson_flips(uv, tris, boundary, scale, &s, sag_tol);
	}
	for _round in 0..40 {
		if tris.len() > ANALYTIC_FACET_BUDGET {
			return Err(format!("patch refinement exceeded the {ANALYTIC_FACET_BUDGET}-facet budget"));
		}
		// Edges with at least one owner above the area floor are the live ones.
		let mut live: std::collections::HashSet<(usize, usize)> = std::collections::HashSet::new();
		for t in tris.iter() {
			if area2(uv, t) > floor2 {
				for k in 0..3 {
					live.insert(edge_key(t[k], t[(k + 1) % 3]));
				}
			}
		}
		// Mark every qualifying edge with its (interned) midpoint.
		let mut marks: HashMap<(usize, usize), usize> = HashMap::new();
		let mut considered: std::collections::HashSet<(usize, usize)> = std::collections::HashSet::new();
		for t in tris.iter() {
			for k in 0..3 {
				let (a, b) = (t[k], t[(k + 1) % 3]);
				let key = edge_key(a, b);
				if boundary.contains(&key) || !live.contains(&key) || !considered.insert(key) {
					continue;
				}
				let d = uv[a] - uv[b];
				if (d.x * scale.x).powi(2) + (d.y * scale.y).powi(2) < min_len2 {
					continue;
				}
				let mid = (uv[a] + uv[b]) * 0.5;
				let s = (eval(mid) - (pos[a] + pos[b]) * 0.5).length();
				if s > sag_tol {
					let m = *by_uv.entry(uv_key(mid)).or_insert_with(|| {
						uv.push(mid);
						pos.push(eval(mid));
						uv.len() - 1
					});
					marks.insert(key, m);
				}
			}
		}
		if marks.is_empty() {
			return Ok(());
		}
		// Longest-edge propagation (Rivara): a triangle with a marked edge also
		// splits its LONGEST splittable edge (3-D chord length), to closure. A
		// non-longest split alone leaves the big triangle in place, and its new
		// midpoint edges converge on a fixed point round after round (measured: a
		// corner ball never converged in 40 rounds); bisecting the longest edge
		// shrinks every owner geometrically, which is what guarantees termination.
		loop {
			let mut added = 0usize;
			for t in tris.iter() {
				let keys = [edge_key(t[0], t[1]), edge_key(t[1], t[2]), edge_key(t[2], t[0])];
				if area2(uv, t) <= floor2 || !keys.iter().any(|k| marks.contains_key(k)) {
					continue;
				}
				let mut longest: Option<((usize, usize), f64)> = None;
				for &k in &keys {
					if boundary.contains(&k) {
						continue;
					}
					let l = (pos[k.0] - pos[k.1]).length_squared();
					if longest.is_none_or(|(_, bl)| l > bl) {
						longest = Some((k, l));
					}
				}
				if let Some((k, _)) = longest {
					if let std::collections::hash_map::Entry::Vacant(slot) = marks.entry(k) {
						let mid = (uv[k.0] + uv[k.1]) * 0.5;
						let m = *by_uv.entry(uv_key(mid)).or_insert_with(|| {
							uv.push(mid);
							pos.push(eval(mid));
							uv.len() - 1
						});
						slot.insert(m);
						added += 1;
					}
				}
			}
			if added == 0 {
				break;
			}
		}
		let mut next: Vec<[usize; 3]> = Vec::with_capacity(tris.len() * 2);
		let len2 = |x: usize, y: usize| {
			let d = uv[x] - uv[y];
			(d.x * scale.x).powi(2) + (d.y * scale.y).powi(2)
		};
		for &[a, b, c] in tris.iter() {
			let mab = marks.get(&edge_key(a, b)).copied();
			let mbc = marks.get(&edge_key(b, c)).copied();
			let mca = marks.get(&edge_key(c, a)).copied();
			match (mab, mbc, mca) {
				(None, None, None) => next.push([a, b, c]),
				(Some(m), None, None) => next.extend([[a, m, c], [m, b, c]]),
				(None, Some(m), None) => next.extend([[a, b, m], [a, m, c]]),
				(None, None, Some(m)) => next.extend([[a, b, m], [m, b, c]]),
				(Some(m1), Some(m2), None) => {
					next.push([m1, b, m2]);
					if len2(a, m2) <= len2(m1, c) {
						next.extend([[a, m1, m2], [a, m2, c]]);
					} else {
						next.extend([[a, m1, c], [m1, m2, c]]);
					}
				}
				(Some(m1), None, Some(m3)) => {
					next.push([a, m1, m3]);
					if len2(m1, c) <= len2(b, m3) {
						next.extend([[m1, b, c], [m1, c, m3]]);
					} else {
						next.extend([[m1, b, m3], [b, c, m3]]);
					}
				}
				(None, Some(m2), Some(m3)) => {
					next.push([m2, c, m3]);
					if len2(a, m2) <= len2(b, m3) {
						next.extend([[a, b, m2], [a, m2, m3]]);
					} else {
						next.extend([[a, b, m3], [b, m2, m3]]);
					}
				}
				(Some(m1), Some(m2), Some(m3)) => next.extend([[a, m1, m3], [m1, b, m2], [m3, m2, c], [m1, m2, m3]]),
			}
		}
		*tris = next;
		let s = |a: usize, b: usize| sag_of(uv, pos, a, b);
		lawson_flips(uv, tris, boundary, scale, &s, sag_tol);
	}
	Err("patch refinement failed to converge within 40 rounds".into())
}

/// Replace the two synthetic seam chords of a bridged band / closed cap ring
/// — chord A `ring[k] → ring[k+1]` and its mirror chord B `ring[n−1] → ring[0]`
/// one period over — by polylines **sampled on the exact surface** at the
/// patch's ring pitch (both copies bit-identical in 3-D, so they still intern
/// to one chain of twins). A straight rim-to-rim chord is only on the surface
/// when it runs along a ruling (a NURBS tube's seam); on a torus band it cuts
/// straight through the tube (sag = the tube radius) and the facets beside it
/// fill two half-discs — measured +4% area on an off-phase torus band, and the
/// same wedge, smaller, beside a sphere cap's rim-to-pole chord.
pub(super) fn sample_synthetic_seams(
	uv: &mut Vec<DVec2>,
	pts3: &mut Vec<DVec3>,
	ring: &[usize],
	k: usize,
	patch: &dyn ParamPatch,
) -> Vec<usize> {
	let n = ring.len();
	let (a_from, a_to) = (ring[k], ring[k + 1]);
	let (b_from, b_to) = (ring[n - 1], ring[0]);
	let scale = patch.chart_scale();
	let d = uv[a_to] - uv[a_from];
	let len = ((d.x * scale.x).powi(2) + (d.y * scale.y).powi(2)).sqrt();
	let pitch = scale.x.min(scale.y) / FULL_TURN_SEGMENTS as f64;
	let samples = ((len / pitch).ceil() as usize).saturating_sub(1).min(4 * FULL_TURN_SEGMENTS);
	if samples == 0 {
		return ring.to_vec();
	}
	let mut chain_a: Vec<usize> = Vec::with_capacity(samples);
	let mut chain_b: Vec<usize> = Vec::with_capacity(samples);
	for j in 1..=samples {
		let f = j as f64 / (samples + 1) as f64;
		let q = uv[a_from].lerp(uv[a_to], f);
		let p = patch.point(q);
		chain_a.push(uv.len());
		uv.push(q);
		pts3.push(p);
	}
	// The mirror chain, traversed the other way, at the mirror chord's own
	// cover coordinates but with chain A's exact positions.
	for j in 1..=samples {
		let f = j as f64 / (samples + 1) as f64;
		let q = uv[b_from].lerp(uv[b_to], f);
		chain_b.push(uv.len());
		uv.push(q);
		pts3.push(pts3[chain_a[samples - j]]);
	}
	let mut out: Vec<usize> = Vec::with_capacity(n + 2 * samples);
	out.extend_from_slice(&ring[..=k]);
	out.extend(chain_a);
	out.extend_from_slice(&ring[k + 1..]);
	out.extend(chain_b);
	out
}
