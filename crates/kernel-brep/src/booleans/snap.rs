// Copyright (c) LMCAD. Licensed under the MIT License.

//! Cut-seam exactness: Newton-project seam vertices that lie on two or three
//! distinct analytic surfaces onto their true intersection, under per-face
//! chart guards, conservative move budgets and an all-or-nothing per-seam
//! coherence rule.

use std::collections::HashMap;

use kernel_core::math::DVec3;

use kernel_core::orient2d;

use crate::geom::{Surface, SurfaceChart};
use crate::topo::FaceInput;

use crate::tol::TJUNCTION_EPS;

use super::triangulate::{dist_point_segment, newell_normal};

/// Below this deviation a seam vertex counts as already ON the exact intersection
/// and is left untouched — so constructed-exact geometry (primitive rims, fillet
/// bands, machine-exact unions) is never perturbed and stays bit-identical.
const SNAP_ALREADY_EXACT: f64 = 1e-9;
/// Newton convergence tolerance (per-surface |signed distance|) for the seam snap.
/// The projector accepts up to 10× this after its iteration cap, so every snapped
/// vertex lies on each of its surfaces to ≤ 1e-11 — well inside test gates at 1e-9.
const SNAP_TOL: f64 = 1e-12;
/// Wedge cross products smaller than this (squared, mm⁴) carry no usable
/// orientation: the wedge is degenerate-collinear and the flip guard of
/// [`snap_seam_vertices`] has nothing to preserve there.
const SNAP_WEDGE_DEGENERATE: f64 = 1e-18;

/// Snap cut-seam vertices onto the exact surface–surface intersection of the
/// operands' analytic surfaces — including, since W5, seams whose snap WARPS
/// the incident curved facets.
///
/// Co-refinement places seam vertices on the operands' tessellation chords: for a
/// curved operand that is off the true intersection curve by up to the facet
/// sagitta (~1.7e-2 mm on a default 24-gon, radius-2 cylinder). Each result vertex
/// whose incident faces carry exactly TWO distinct analytic surfaces is
/// Newton-projected onto `f = g = 0` with the same min-norm projector as the SSI
/// tracer ([`crate::ssi::project`]); a vertex on exactly THREE distinct surfaces
/// (a seam corner — e.g. a cut plane pair crossing a curved wall, or a cap rim
/// meeting a quadric∩quadric seam) is solved fully determined
/// ([`crate::ssi::project3`]). Plane∩plane-only vertices are already exact and
/// are skipped, as are vertices on a single surface or on 4+ (ambiguous).
///
/// **W3 → W5.** W3's load-bearing *planarity contract* accepted a move only if
/// every incident facet stayed exactly planar (seam corners on cylinder
/// generators: ⟂ cuts, axis-parallel keyways/notches), because a WARPED polygon
/// could fold under projection-plane ear-clipping in the next boolean of a chain
/// — folded, double-covered triangles whose surplus directed edges cannot
/// twin-pair (fuzz-measured: unrestricted snapping 88.5%). W5 removes the fold
/// at its root: a warped curved-tagged face is now clipped in its surface's
/// PARAMETER SPACE ([`SurfaceChart`] via [`face_clip_p2`]), where a boundary
/// that bounds a simple region of the surface cannot fold. The guards became:
///
/// - **chart guard** ([`chart_snap_safe`]) instead of the planarity guard for
///   every face whose surface the triangulator owns ([`snap_chart_surface`]):
///   the proposed ring must chart inside the injective domain, stay simple,
///   keep its area orientation, and not be (or become) thinner than the heal
///   scale — inflating a zero-thickness sliver face into a finite fin
///   self-overlaps the boundary (fuzz seed 83894724543990). Planes keep the
///   planarity + 3-D flip guards verbatim (the intersection curve lies IN every
///   incident plane, so true seam moves cost planes nothing);
/// - **seam coherence**, all-or-nothing per surface PAIR: a seam may move only
///   if every vertex on it snaps, already lies on the exact intersection
///   (≤ 2×[`TJUNCTION_EPS`] per surface), or is a mid-edge chord sample whose
///   host stretches stay straight under the proposed moves. Vertex-level
///   accept/reject otherwise leaves a seam HALF-snapped — a zigzag between the
///   true curve and the chords whose two sides overlap in an unclassifiable
///   near-zero-thickness fin;
/// - the conservative W3 **move budgets stay in force for sphere/cone/torus
///   vertices** (≤ a tenth of the shortest incident boundary edge, wedge-margin
///   alternative for 3-surface corners): their facet sagitta is 10–20× a
///   cylinder's, and freeing it measurably breaks chains (deep N=2000: 99.9%
///   with budget-free spheres, 100.0% with this split). Plane/cylinder-only
///   vertices snap with the full sagitta-scale budget (≤ 2× the smallest
///   incident curved chord band), and their straight-seam chord samples snap
///   WITH their seam — a plane-anchored chord rim still never bends mid-edge
///   (the θ-rectangular `exact_volume` premise).
///
/// Unchanged phase-1 gates: the vertex lies within each incident surface's
/// measured chord band (it IS a seam vertex of those surfaces); it is off the
/// intersection by more than [`SNAP_ALREADY_EXACT`] and the move exceeds the
/// weld/heal noise scale (2× [`TJUNCTION_EPS`] — nudging by ~1e-7..1e-6 only
/// creates clusters the welder cannot merge); Newton must converge (tangential
/// contact ⇒ parallel gradients ⇒ skip). All rejections run in whole-set sweeps
/// to a fixed point — deterministic, and each rejection re-checks the
/// neighbourhood it re-exposes. Only vertex *positions* change — ids and
/// topology are untouched, so a closed manifold result stays closed and
/// manifold. Chain-fuzz measured at every step (deep N=2000 ×3 byte-identical,
/// floors 98): the full W5 variant table is in `ROBUSTNESS.md`.
pub(super) fn snap_seam_vertices(verts: &mut [DVec3], faces: &[FaceInput]) {
	// Distinct surfaces across the result (geometric dedup) with their measured
	// chord bands; `face_surf` maps each face to its surface index.
	let mut surfs: Vec<Surface> = Vec::new();
	let mut bands: Vec<f64> = Vec::new();
	let mut face_surf: Vec<usize> = Vec::with_capacity(faces.len());
	let mut any_curved = false;
	for f in faces {
		let si = match surfs.iter().position(|s| s.same_locus(&f.surface, 1e-9)) {
			Some(i) => i,
			None => {
				surfs.push(f.surface);
				bands.push(0.0);
				surfs.len() - 1
			}
		};
		face_surf.push(si);
		if !matches!(f.surface, Surface::Plane { .. }) {
			any_curved = true;
			let n = f.boundary.len();
			for i in 0..n {
				let mid = (verts[f.boundary[i] as usize] + verts[f.boundary[(i + 1) % n] as usize]) * 0.5;
				bands[si] = bands[si].max(f.surface.signed_value(mid).abs());
			}
		}
	}
	if !any_curved {
		return; // purely planar arrangement — every seam is already exact
	}
	// Distinct incident surfaces per vertex; the corner test (a vertex within
	// TJUNCTION_EPS of the segment joining its ring neighbours on ANY incident
	// boundary is a chord sample, not a corner); and the two pre-snap move
	// budgets: the shortest incident boundary edge and the wedge margin (the
	// smallest distance to the line through the ring neighbours — the distance
	// at which the wedge `(prev, v, next)` would invert).
	let mut incident: Vec<Vec<usize>> = vec![Vec::new(); verts.len()];
	let mut mid_edge_sample = vec![false; verts.len()];
	let mut min_edge = vec![f64::INFINITY; verts.len()];
	let mut wedge_margin = vec![f64::INFINITY; verts.len()];
	// Ring-neighbour pairs of every mid-edge sample — the endpoints of the
	// straight stretches it subdivides, for the seam-coherence "stretch stays
	// straight" test below (filled in a second pass once the flags are known).
	let mut stretch_hosts: Vec<Vec<(u32, u32)>> = vec![Vec::new(); verts.len()];
	for (fi, f) in faces.iter().enumerate() {
		let n = f.boundary.len();
		for (i, &v) in f.boundary.iter().enumerate() {
			let list = &mut incident[v as usize];
			if !list.contains(&face_surf[fi]) {
				list.push(face_surf[fi]);
			}
			let prev = f.boundary[(i + n - 1) % n];
			let next = f.boundary[(i + 1) % n];
			if prev == v || next == v {
				continue;
			}
			let (pv, pa, pb) = (verts[v as usize], verts[prev as usize], verts[next as usize]);
			min_edge[v as usize] = min_edge[v as usize].min((pb - pv).length()).min((pa - pv).length());
			let dir = pb - pa;
			let margin = if dir.length_squared() > 1e-24 { (pv - pa).cross(dir).length() / dir.length() } else { (pv - pa).length() };
			wedge_margin[v as usize] = wedge_margin[v as usize].min(margin);
			if prev != next && dist_point_segment(pv, pa, pb) <= TJUNCTION_EPS {
				mid_edge_sample[v as usize] = true;
			}
		}
	}
	for f in faces.iter() {
		let n = f.boundary.len();
		for (i, &v) in f.boundary.iter().enumerate() {
			if !mid_edge_sample[v as usize] {
				continue;
			}
			let prev = f.boundary[(i + n - 1) % n];
			let next = f.boundary[(i + 1) % n];
			if prev != v && next != v {
				stretch_hosts[v as usize].push((prev, next));
			}
		}
	}
	// Phase 1 — per-vertex Newton targets through every gate except the
	// planarity/flip guards (those need the full proposed configuration).
	let mut target: Vec<Option<DVec3>> = vec![None; verts.len()];
	let mut any_target = false;
	for (vi, list) in incident.iter().enumerate() {
		// A snappable seam vertex sits on two or three distinct surfaces, every
		// curved one of a kind the parameter-space triangulator owns
		// ([`snap_chart_surface`]). All-plane vertices are already exact; 4+
		// surfaces are ambiguous. Since W5 a vertex needs NO plane among its
		// surfaces (quadric∩quadric seams — cylinder∪cylinder — snap too): the
		// warped facets a plane-less snap leaves behind are exactly what the
		// chart triangulator exists to clip.
		let n_planes = list.iter().filter(|&&i| matches!(surfs[i], Surface::Plane { .. })).count();
		let kinds_ok =
			list.iter().all(|&i| matches!(surfs[i], Surface::Plane { .. } | Surface::Cylinder { .. }) || snap_chart_surface(&surfs[i]));
		if !(list.len() == 2 || list.len() == 3) || (n_planes == 0 && !SNAP_ALLOW_PLANELESS) || n_planes == list.len() || !kinds_ok {
			continue;
		}
		// Vertices whose surfaces are all planes/CYLINDERS take the FULL W5
		// relaxation: the phase-2 guards judge their post-move configuration
		// exactly (the move lies in each incident plane, so planarity is free;
		// cylinder faces get the chart guard), so the W3-era pre-filters below
		// relax for them. Sphere/cone/torus vertices keep the W3 budgets even
		// though their faces are chart-clipped: their facet sagitta is typically
		// 10–20× a cylinder's (a 16×12 fuzz sphere's band is ~0.3 mm), and
		// fuzz-measured at that scale the freed moves interleave neighbouring
		// faces faster than weld/T-junction healing can absorb (deep N=2000:
		// budget-free spheres 99.9%, this split 100.0% — see ROBUSTNESS.md W5).
		let full_relax = list.iter().all(|&i| matches!(surfs[i], Surface::Plane { .. } | Surface::Cylinder { .. }));
		// Chord samples merely subdivide a straight facet-boundary stretch and stay
		// put where a PLANE anchors the seam: the chord rim is what the
		// θ-rectangular bulge corrections of `exact_volume` integrate over (see the
		// doc above). On a pure-CYLINDER seam (quadric∩quadric) there is no exact
		// rectangular patch to protect — and leaving a sample at chord∩chord depth
		// while its seam corners snap would zigzag the seam, which the coherence
		// rule below then (rightly) vetoes wholesale — so there the samples snap
		// WITH their seam.
		if mid_edge_sample[vi] && (n_planes > 0 || !full_relax) {
			continue;
		}
		let p = verts[vi];
		let mut curved_band = f64::INFINITY;
		let mut max_dev = 0.0f64;
		let mut in_band = true;
		for &i in list {
			let dev = surfs[i].signed_value(p).abs();
			in_band &= dev <= bands[i].max(TJUNCTION_EPS);
			if !matches!(surfs[i], Surface::Plane { .. }) {
				curved_band = curved_band.min(bands[i]);
			}
			max_dev = max_dev.max(dev);
		}
		if !in_band || max_dev <= SNAP_ALREADY_EXACT {
			continue;
		}
		let proj = match list.as_slice() {
			[a, b] => crate::ssi::project(&surfs[*a], &surfs[*b], p, SNAP_TOL),
			[a, b, c] => crate::ssi::project3(&surfs[*a], &surfs[*b], &surfs[*c], p, SNAP_TOL),
			_ => None,
		};
		if let Some(s) = proj {
			// Pre-snap move budget. (a) ≤ 2× the smallest incident CURVED band: the
			// move bends each curved facet by at most ~2× the sagitta its analytic
			// tag already promises, and a glancing intersection cannot yank the
			// vertex (the plane faces are warp-free — the intersection curve lies IN
			// every incident plane). (b) for a vertex whose curved surfaces are NOT
			// all chart-owned, additionally ≤ a tenth of the shortest incident
			// boundary edge, so no fragment is bent at its own scale; a
			// fully-determined THREE-surface corner (project3, a genuine turn on
			// every ring) may alternatively spend half its wedge margin — the
			// distance at which a wedge would invert — which is what admits a corner
			// sitting next to a short co-refinement stub edge (the keyway corner).
			// A plane/cylinder-only vertex needs no edge pre-cap: bending a fragment
			// at its own scale is dangerous exactly when the bend must stay planar,
			// and the chart/flip guards of phase 2 judge any inversion the move
			// could cause exactly, face by face (a quadric∩quadric seam vertex
			// routinely needs its full sagitta-scale move next to a short
			// seam-polyline edge).
			let edge_budget = 0.1 * min_edge[vi];
			let budget = if full_relax {
				(2.0 * curved_band).min(SNAP_MOVE_CAP)
			} else {
				(2.0 * curved_band).min(if list.len() == 3 { edge_budget.max(0.5 * wedge_margin[vi]) } else { edge_budget })
			};
			// Also skip MICRO-moves below the weld/heal scale: a vertex < ~1e-6 off
			// the exact intersection is co-refinement/heal noise, and nudging it
			// creates near-coincident vertex clusters (1e-7..1e-6 apart) that the
			// welder (1e-7) cannot merge and T-junction healing (4e-7) cannot
			// close — sliver micro-holes in the next boolean of a chain. Genuine
			// chord-seam vertices sit ≥ ~1e-3 off; nothing of value is skipped.
			let move_len = (s - p).length();
			if move_len <= budget && move_len > 2.0 * TJUNCTION_EPS {
				target[vi] = Some(s);
				any_target = true;
			}
		}
	}
	if !any_target {
		return;
	}
	// SEAM COHERENCE groups (W5): all vertices lying on the seam of one surface
	// PAIR, keyed by the sorted surface-index pair (only pairs with a curved
	// member — plane∩plane is always exact). A pair's seam may move only
	// ALL-OR-NOTHING: if any vertex on it can neither snap (no surviving target)
	// nor already sits on the exact intersection to within the weld/heal scale,
	// every target on that seam is dropped. Without this, vertex-level
	// accept/reject left seams PARTIALLY snapped — a zigzag polyline between the
	// true curve and the chords — and the faces on the two sides of the zigzag
	// (one following the snapped corners, one still holding unmoved chord-depth
	// junction vertices, e.g. where a third face crosses the seam mid-line)
	// OVERLAP in a near-zero-thickness fin that the NEXT boolean's arrangement
	// cannot classify (fuzz seed 83894724543990, op 4 — a 4-half-edge
	// non-manifold sandwich along the snapped generator).
	let mut pair_groups: Vec<((usize, usize), Vec<usize>)> = {
		let mut map: HashMap<(usize, usize), Vec<usize>> = HashMap::new();
		for (vi, list) in incident.iter().enumerate() {
			for i in 0..list.len() {
				for j in i + 1..list.len() {
					let (a, b) = (list[i].min(list[j]), list[i].max(list[j]));
					if matches!(surfs[a], Surface::Plane { .. }) && matches!(surfs[b], Surface::Plane { .. }) {
						continue;
					}
					map.entry((a, b)).or_default().push(vi);
				}
			}
		}
		map.into_iter().collect()
	};
	pair_groups.sort_unstable_by_key(|(k, _)| *k);
	// On the exact (a, b) intersection to within the scale weld + T-junction
	// healing already absorb — the same floor the phase-1 micro-move skip uses,
	// so a skipped near-exact vertex never vetoes its seam.
	let near_pair = |vi: usize, a: usize, b: usize, verts: &[DVec3]| -> bool {
		let p = verts[vi];
		surfs[a].signed_value(p).abs() <= 2.0 * TJUNCTION_EPS && surfs[b].signed_value(p).abs() <= 2.0 * TJUNCTION_EPS
	};
	// Phase 2 — face guards (planarity/flip, or the chart guard for faces the
	// parameter-space triangulator owns) + seam coherence, iterated to a fixed
	// point. All are evaluated with every surviving candidate at its proposed
	// target (so a seam snapping coherently is judged as a whole); a violating
	// face/wedge/pair rejects ALL its moved members, and rejection can re-expose
	// a neighbour to the pre-snap position, so sweep until clean. The candidate
	// set only shrinks, so this terminates; sweeps are whole-set per pass and
	// faces/boundaries are deterministic, so the outcome is run-deterministic.
	loop {
		let pos = |v: u32| target[v as usize].unwrap_or(verts[v as usize]);
		// A mid-edge chord SAMPLE is coherent with its seam exactly when every
		// straight stretch it subdivides STAYS straight under the proposed moves
		// (its hosts' endpoints unmoved, or moved along the stretch): then it is
		// construction geometry (a facet-chord interior point — e.g. a cap-rim
		// chord sample at the expected sagitta depth), not a stranded seam
		// vertex. If a host stretch BENDS away from it, the sample would be left
		// as a zigzag spike off the snapped seam — the partial-snap fin — and it
		// must veto.
		let stretch_stays = |vi: usize| -> bool {
			!stretch_hosts[vi].is_empty()
				&& stretch_hosts[vi].iter().all(|&(a, b)| dist_point_segment(verts[vi], pos(a), pos(b)) <= TJUNCTION_EPS)
		};
		let mut reject: Vec<u32> = Vec::new();
		for ((a, b), members) in &pair_groups {
			if members.iter().all(|&vi| target[vi].is_some() || near_pair(vi, *a, *b, verts) || stretch_stays(vi)) {
				continue;
			}
			reject.extend(members.iter().copied().filter(|&vi| target[vi].is_some()).map(|vi| vi as u32));
		}
		for f in faces.iter() {
			let n = f.boundary.len();
			let f_moved: Vec<u32> = f.boundary.iter().copied().filter(|&m| target[m as usize].is_some()).collect();
			if f_moved.is_empty() {
				continue;
			}
			let prop: Vec<DVec3> = f.boundary.iter().map(|&v| pos(v)).collect();
			let cur: Vec<DVec3> = f.boundary.iter().map(|&v| verts[v as usize]).collect();
			// A face whose surface the PARAMETER-SPACE triangulator owns may leave
			// its chord plane — every later clip of it runs in its surface chart
			// ([`SurfaceChart`]), where warp is invisible. The planarity/flip guards
			// are replaced by the chart guard: the proposed ring must stay
			// clip-safe in that chart (every vertex inside the injective domain,
			// every wedge keeping its 2-D orientation, no self-crossing). W3's
			// planarity contract remains in force verbatim for every face whose
			// surface the chart does NOT own (planes stay true planes).
			if snap_chart_surface(&f.surface) {
				if !chart_snap_safe(&f.surface, &cur, &prop) {
					reject.extend(f_moved.iter().copied());
				}
				continue;
			}
			// PLANARITY guard: with every candidate at its proposed target, the face
			// must stay as planar as it was (within round-off), so the result remains
			// a true planar arrangement. ⟂ and axis-parallel cuts pass — their seam
			// vertices land on cylinder GENERATORS, and a facet with corners on two
			// generators is exactly planar. An oblique or partial-depth cut whose
			// seam endpoint lands mid-facet would WARP the facet by up to the
			// sagitta; ear-clipping a warped polygon in a projection plane can fold
			// (double-cover) and the next boolean in a chain then stitch-explodes —
			// the measured post-snap fuzz failure class. Such snaps are rejected;
			// those seams keep their chord vertices (the stated residual contract).
			let planarity = |poly: &[DVec3]| -> f64 {
				let nrm = newell_normal(poly);
				if nrm.length_squared() < 0.5 {
					return f64::INFINITY;
				}
				poly.iter().map(|&p| (p - poly[0]).dot(nrm).abs()).fold(0.0, f64::max)
			};
			if planarity(&prop) > planarity(&cur) + 10.0 * SNAP_ALREADY_EXACT {
				reject.extend(f_moved.iter().copied());
				continue;
			}
			// FLIP guard: no boundary wedge whose corner or neighbours moved may
			// reverse its cross-product orientation (an inverted sliver is sliver
			// debris for the next boolean even inside a still-planar face).
			for (i, &v) in f.boundary.iter().enumerate() {
				let prev = f.boundary[(i + n - 1) % n];
				let next = f.boundary[(i + 1) % n];
				let moved: Vec<u32> = [prev, v, next].into_iter().filter(|&m| target[m as usize].is_some()).collect();
				if moved.is_empty() {
					continue;
				}
				let (a0, v0, b0) = (verts[prev as usize], verts[v as usize], verts[next as usize]);
				let w0 = (v0 - a0).cross(b0 - v0);
				if w0.length_squared() <= SNAP_WEDGE_DEGENERATE {
					continue; // collinear pre-snap: no orientation to preserve
				}
				let w1 = (pos(v) - pos(prev)).cross(pos(next) - pos(v));
				if w0.dot(w1) <= 0.0 {
					reject.extend(moved);
				}
			}
		}
		if reject.is_empty() {
			break;
		}
		for v in reject {
			target[v as usize] = None;
		}
	}
	for (vi, t) in target.into_iter().enumerate() {
		if let Some(s) = t {
			verts[vi] = s;
		}
	}
}

/// The curved surface kinds whose snapped (warped) facets the W5
/// **parameter-space triangulator** owns end to end — [`face_clip_p2`] /
/// `tessellate_curved_verbatim` clip such faces in their [`SurfaceChart`], so
/// [`snap_seam_vertices`] may let them leave their chord planes (the chart guard
/// [`chart_snap_safe`] replaces W3's planarity guard there). Admission is
/// per-kind and fuzz-measured on the deep N=2000 corpus ×3 (see the W5 variant
/// table in `ROBUSTNESS.md`); a kind that cannot hold the 100.0% rate stays out
/// and keeps the W3 chord contract.
fn snap_chart_surface(s: &Surface) -> bool {
	// All four analytic curved kinds: each held the deep corpus at 100.0% ×3
	// (cones/tori enter the corpus only through fillet bands; their admission
	// additionally rests on the chart unit tests and the in-tree torus/fillet
	// suites). `false` for a kind would restore W3's planarity guard for its
	// faces — the honest fallback if a kind ever fails to hold the rate.
	matches!(s, Surface::Cylinder { .. } | Surface::Sphere { .. } | Surface::Cone { .. } | Surface::Torus { .. })
}

/// Whether seam vertices with NO plane among their surfaces (quadric∩quadric —
/// e.g. cylinder∪cylinder) may snap. Requires the parameter-space triangulator:
/// a plane-less snap warps EVERY incident facet.
const SNAP_ALLOW_PLANELESS: bool = true;

/// Absolute cap (mm) on a budget-free (plane/cylinder-only) seam move — the same
/// absolute-tolerance convention as EPS/WELD_EPS/TJUNCTION_EPS. The chart guards
/// judge each FACE exactly, but a sagitta-scale move also interleaves the warped
/// face with its NEIGHBOURS, and beyond ~this depth the weld (1e-7) and
/// T-junction healing (4e-7) of the next boolean measurably stop absorbing the
/// disagreement: a coarse 11-gon r=7.5 fuzz cylinder (sagitta 0.30 mm) regressed
/// the Level-9 N=10 000 corpus until capped, while every practical tessellation
/// this kernel emits (24-gon at r ≤ ~25 mm, sagitta ≤ 0.05) snaps in full.
/// Seams needing larger moves keep their chords — the honest boundary.
const SNAP_MOVE_CAP: f64 = 0.05;

/// Whether the proposed (post-snap) boundary of a chart-owned curved face stays
/// safe to ear-clip in its surface's parameter space — the W5 replacement for
/// the planarity guard on such faces. Anchored on the CURRENT ring (`cur` never
/// changes between fixed-point sweeps, so the chart — and hence the verdict
/// sequence — is deterministic). Four conditions, all conservative:
///
/// - the chart exists and maps **every** current and proposed vertex inside its
///   injective domain (a vertex at the gnomonic horizon / on an axis refuses);
/// - the CURRENT ring is not thinner than the heal scale (chart area ≤ perimeter
///   × [`TJUNCTION_EPS`] — e.g. the zero-thickness sliver face bridging two
///   differently-subdivided copies of one seam line). Such a ring carries no
///   orientation to judge a move by, and moving its corners INFLATES the sliver
///   into a finite-width fin overlapping its neighbour faces — a
///   self-overlapping boundary that explodes the NEXT boolean's arrangement
///   (fuzz seed 83894724543990): a snap may realign a face's vertices on its
///   surface, never grow new area. The PROPOSED ring must clear the same bar
///   (a move may not thin a real face into heal-scale debris);
/// - the proposed ring keeps the current ring's signed-area ORIENTATION (a ring
///   turning inside-out would flip the face winding);
/// - the proposed ring stays **simple** — no proper crossing between
///   non-adjacent edges (O(n²) over short boundaries). Ear-clipping triangulates
///   any simple ring exactly, so individual wedges are free to change convexity
///   — a chord-polyline wedge at the sagitta scale legitimately REVERSES when
///   its vertices land on the true intersection curve, which is why this guard
///   deliberately has no per-wedge flip condition (W3's 3-D flip guard stays in
///   force for non-chart faces, where an inverted sliver inside a still-planar
///   face is real debris).
pub(super) fn chart_snap_safe(surface: &Surface, cur: &[DVec3], prop: &[DVec3]) -> bool {
	let Some(chart) = SurfaceChart::new(surface, cur) else {
		return false;
	};
	let (Some(uv_cur), Some(uv_prop)) = (chart.uv_ring(cur), chart.uv_ring(prop)) else {
		return false;
	};
	let n = uv_prop.len();
	let ring_area2_perim = |uv: &[glam::DVec2]| {
		let (mut area2, mut perim) = (0.0, 0.0);
		for i in 0..n {
			let (a, b) = (uv[i], uv[(i + 1) % n]);
			area2 += a.perp_dot(b);
			perim += (b - a).length();
		}
		(area2, perim)
	};
	let (cur_area2, cur_perim) = ring_area2_perim(&uv_cur);
	let (prop_area2, prop_perim) = ring_area2_perim(&uv_prop);
	if cur_area2.abs() * 0.5 <= cur_perim * TJUNCTION_EPS
		|| prop_area2.abs() * 0.5 <= prop_perim * TJUNCTION_EPS
		|| cur_area2 * prop_area2 <= 0.0
	{
		return false;
	}
	// Simplicity of the proposed ring: no proper (interior×interior) crossing
	// between non-adjacent edges.
	for i in 0..n {
		let (a, b) = (uv_prop[i], uv_prop[(i + 1) % n]);
		for j in i + 1..n {
			if j == i || (j + 1) % n == i || (i + 1) % n == j {
				continue;
			}
			let (c, d) = (uv_prop[j], uv_prop[(j + 1) % n]);
			let o = |p: glam::DVec2, q: glam::DVec2, r: glam::DVec2| orient2d([p.x, p.y], [q.x, q.y], [r.x, r.y]);
			if o(a, b, c) * o(a, b, d) < 0.0 && o(c, d, a) * o(c, d, b) < 0.0 {
				return false;
			}
		}
	}
	true
}
