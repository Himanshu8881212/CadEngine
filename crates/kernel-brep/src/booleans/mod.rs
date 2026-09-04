// Copyright (c) LMCAD. Licensed under the MIT License.

// Copyright (c) LMCAD. Licensed under the MIT License.

//! Exact constructive-solid-geometry booleans on **planar-faced (polyhedral)**
//! B-rep [`Solid`]s — union, difference and intersection.
//!
//! The algorithm is the classical mesh-arrangement boolean specialised to the
//! exact B-rep representation, and it is **general**: it works for any valid
//! closed planar solid (cuboid, [`extrude`](crate::extrude)d prism, any convex
//! or non-convex polyhedron), never for one specific shape.
//!
//! ## Pipeline
//! 1. **Triangulate** both operands. Each B-rep face is planar, so ear-clipping
//!    is exact and every emitted triangle lies in its face's supporting plane.
//!    A face with inner (hole) loops is bridged into one simple ring first, so a
//!    hole is a true opening, never silently filled. Boundaries are first stripped
//!    of redundant collinear micro-subdivisions (T-junction chains a previous
//!    boolean healed into the faces) so chained ops do not accumulate needle
//!    fans — see [`chain_redundant_vertices`]. The triangle is the unit of
//!    work; because the input faces are planar this loses no geometry (re-merging
//!    coplanar triangles afterwards restores faces).
//! 2. **Co-refine.** Every triangle of A is split along its intersection with
//!    every triangle of B (plane–plane SSI gives a line; clipped to both triangle
//!    polygons it gives a segment), and vice-versa. After this pass no triangle
//!    of A straddles the surface of B, so each fragment is wholly inside or
//!    wholly outside the other solid.
//! 3. **Classify** every fragment by its centroid against the *other* solid using
//!    a robust ray-cast point-in-polyhedron test on the other solid's triangles.
//! 4. **Select & orient** fragments per the boolean rule (union keeps the parts
//!    of each surface outside the other; intersection keeps the inside parts;
//!    difference keeps A-outside plus B-inside with B's facets flipped).
//! 5. **Stitch** the kept triangles back into a closed half-edge [`Solid`] via
//!    [`Solid::from_faces`], welding coincident vertices to a tolerance so that
//!    twin half-edges pair up. Sub-tolerance slivers are dropped, T-junctions are
//!    healed by vertex insertion, and coplanar adjacent triangles are merged back
//!    into maximal planar faces — each merged face is verified to ear-clip to
//!    exactly its triangles' area (else it is re-expanded), so chained booleans
//!    (a second hole into an already-drilled face, a cut crossing a bored wall)
//!    stay valid instead of exploding.
//!
//! ## Scope and the curved-seam contract
//! The **arrangement** itself is planar: a curved input face enters as its
//! tessellated planar facets, carrying its analytic [`Surface`] tag by value.
//! Three post-passes then restore curved exactness where it is well-defined:
//!
//! - **Cut fragments keep their analytic tags** ([`recover_faces`]): a clipped
//!   bore band is still `Surface::Cylinder`, so adaptive tessellation, section
//!   queries and `exact_volume` see the true surface through the cut.
//! - **Cut-seam vertices land on the TRUE surface–surface intersection**
//!   ([`snap_seam_vertices`]): a seam vertex on two or three distinct analytic
//!   surfaces is Newton-projected onto their exact intersection via the
//!   [`crate::ssi`] machinery, to ≤ 1e-11 per surface — not left on the ~1e-2
//!   facet chords the raw arrangement produces. Since W5 this covers seams that
//!   WARP their incident facets — oblique cuts, cut rims ending mid-facet, and
//!   plane-less quadric∩quadric seams (cylinder∪cylinder) — because warped
//!   curved-tagged faces are no longer ear-clipped in a projection plane (where
//!   they fold) but in their surface's PARAMETER SPACE ([`SurfaceChart`]; W3's
//!   planarity contract existed precisely because the projection-plane clip of
//!   warped polygons exploded chained booleans). Snapping stays fuzz-guarded by
//!   per-face chart guards ([`chart_snap_safe`]) and an all-or-nothing per-seam
//!   coherence rule; the conservative W3 move budgets remain in force for
//!   sphere/cone/torus vertices, whose 10–20× larger facet sagitta measurably
//!   breaks chains when freed (deep N=2000 corpus — `ROBUSTNESS.md` W5). Seam
//!   EDGES between snapped vertices remain chords of the true curve (the
//!   polyline is vertex-exact, not arc-exact).
//! - **Plane∩quadric cut seams carry exact [`Curve`] tags**
//!   ([`attach_seam_curves`]): circles / ellipses / generator lines from
//!   [`Surface::plane_section`], on snapped cut seams as well as surviving
//!   construction rims — since W5 including the oblique-cut ELLIPSE, whose seam
//!   vertices now land on the section curve. An edge is tagged only when both
//!   endpoints lie ON the section curve, so a chord-accurate seam (a
//!   coherence-vetoed snap) is honestly left untagged, and a quadric∩quadric
//!   seam has no conic closed form and remains an untagged vertex-exact
//!   polyline.
//!
//! Volume: for purely planar solids the result is exact to floating-point (e.g.
//! the union volume of two overlapping boxes matches inclusion–exclusion to
//! ~1e-9). With curved faces, `exact_volume`'s analytic bulge corrections are
//! machine-exact whenever each cut facet remains a θ-rectangular patch of its
//! surface — ⟂ and axis-parallel cylinder cuts qualify, and seam snapping keeps
//! them exact through chained cuts (the keyway identity closes to round-off);
//! oblique conic cuts and quadric∩quadric seams remain facet-level
//! approximations (the corrections assume chord facets, which a snapped warped
//! facet is not — measured honestly in
//! `quadric_quadric_union_volume_stays_facet_level_and_beats_faceted`) —
//! stated, never silent.
//!
//! ## Module map
//! One pipeline stage per file, in the order the driver runs them:
//! [`triangulate`] (step 1) → [`arrange`] (step 2, co-refinement) → [`classify`]
//! (steps 3–4) → [`stitch`] (step 5), which calls [`faces`] for coincident-facet
//! cancellation, planar-face recovery and T-junction healing, and [`snap`] for
//! the cut-seam projection onto the true surface–surface intersection. [`par`]
//! holds the scheduling-only threading layer the pure-map stages share. This
//! file keeps the driver, the [`Op`] rule, the [`Tri`] carrier and the
//! seam-curve tagging pass.

mod arrange;
mod classify;
mod faces;
mod par;
mod snap;
mod stitch;
mod triangulate;

use kernel_core::math::DVec3;

use crate::geom::{Curve, Surface};
use crate::topo::{FaceName, FaceSource, Solid};

use crate::tol::EPS;

use self::arrange::co_refine;
use self::classify::classify_select;
use self::par::brep_workers;
use self::stitch::stitch;
use self::triangulate::triangulate_solid;

pub use self::par::par_items_processed;

/// Which boolean to evaluate.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Op {
	Union,
	Difference,
	Intersection,
}

/// A triangle carrying the supporting plane it came from (so coplanar fragments
/// can later be merged back into a maximal planar face).
#[derive(Clone, Copy, Debug)]
struct Tri {
	v: [DVec3; 3],
	/// Unit outward normal of the source face (orientation reference).
	normal: DVec3,
	/// Persistent name of the operand face this triangle (and its fragments) came
	/// from — propagated through co-refinement so result faces can be re-identified.
	source: FaceName,
	/// The analytic surface of the operand face this triangle tessellates, carried
	/// BY VALUE (always in the operands' original frame — the recentred arrangement
	/// frame never reads it). A name-keyed surface lookup is ambiguous in a CHAINED
	/// boolean: an operand that is itself a boolean result carries `OperandA/B` face
	/// names from ITS OWN operands, which collide with the other operand's names —
	/// that mis-tagged e.g. a first bore's wall with the second bore's cylinder and
	/// made result volumes flake run to run.
	surface: Surface,
}

impl Tri {
	fn centroid(&self) -> DVec3 {
		(self.v[0] + self.v[1] + self.v[2]) / 3.0
	}

	/// Twice the area vector (length = 2·area, direction = geometric normal).
	fn area_vec(&self) -> DVec3 {
		(self.v[1] - self.v[0]).cross(self.v[2] - self.v[0])
	}

	fn area(&self) -> f64 {
		self.area_vec().length() * 0.5
	}

	fn is_degenerate(&self) -> bool {
		self.area() <= EPS * EPS
	}
}

/// The exact union `A ∪ B` of two planar-faced solids.
pub fn union(a: &Solid, b: &Solid) -> Solid {
	boolean(a, b, Op::Union)
}

/// The exact difference `A − B` of two planar-faced solids (material of `a` with
/// the material of `b` removed).
pub fn difference(a: &Solid, b: &Solid) -> Solid {
	boolean(a, b, Op::Difference)
}

/// The exact intersection `A ∩ B` of two planar-faced solids (the material common
/// to both).
pub fn intersection(a: &Solid, b: &Solid) -> Solid {
	boolean(a, b, Op::Intersection)
}

// --- Driver ------------------------------------------------------------------

fn boolean(a: &Solid, b: &Solid, op: Op) -> Solid {
	// Worker budget for the pure-map stages, read ONCE per invocation (see the
	// parallelism section above; `LMCAD_BREP_THREADS=1` is the exact legacy
	// sequential schedule — same code, trivial iteration).
	let workers = brep_workers();
	let mut tris_a = triangulate_solid(a, FaceSource::OperandA);
	let mut tris_b = triangulate_solid(b, FaceSource::OperandB);
	if tris_a.is_empty() || tris_b.is_empty() {
		// One operand is empty: union/difference is A, intersection is empty.
		return match op {
			Op::Union | Op::Difference => a.clone(),
			Op::Intersection => Solid::default(),
		};
	}

	// Far from the origin the arrangement's fixed *absolute* tolerances (EPS,
	// WELD_EPS) fall below the f64 ulp (ulp ≈ 1e-8 at 1e8) and coincidence tests
	// collapse. Re-centre the operands so the arrangement runs in full precision,
	// then shift the kept fragments back before stitching. This only engages in the
	// far field: in place (`ZERO`) the translate is an exact no-op, so near-origin
	// results — including EPS-sensitive off-axis geometry — are byte-identical to the
	// un-centred path. The un-centred arrangement is reliable to ~1e7. Normals are
	// translation-invariant, so only vertices move.
	let bbox_center = tris_bbox_center(&tris_a, &tris_b);
	let center = if bbox_center.abs().max_element() > 1e7 { bbox_center } else { DVec3::ZERO };
	translate_tris(&mut tris_a, -center);
	translate_tris(&mut tris_b, -center);

	// Co-refine each operand against the other so no fragment straddles the other
	// solid's surface.
	let frags_a = co_refine(&tris_a, &tris_b, workers);
	let frags_b = co_refine(&tris_b, &tris_a, workers);

	// Classify + select. Each kept fragment already carries its operand-face name.
	// A-side kept fragments precede B-side ones, exactly as the sequential
	// appends always did.
	let mut kept: Vec<Tri> = classify_select(&frags_a, &tris_b, op, false, workers);
	kept.extend(classify_select(&frags_b, &tris_a, op, true, workers));

	// Shift the kept fragments back to the operands' original location, then stitch
	// in that frame so the output vertices and their supporting surfaces are derived
	// consistently (stitching in the centred frame and translating the finished
	// solid would round vertices and surface anchors apart and crack the tessellation).
	// Each fragment carries its operand face's analytic surface BY VALUE (in this
	// original frame), so an uncut curved facet keeps its Surface::Cylinder/Sphere tag
	// on the result without any name-keyed lookup.
	translate_tris(&mut kept, center);
	let mut solid = stitch(&kept);
	attach_seam_curves(&mut solid);
	solid
}

/// Tag each edge that bounds a planar face and a curved face with the exact analytic
/// intersection [`Curve`] of that plane with the curved surface (a cylinder/sphere/cone
/// cut ⟂ its axis → a [`Curve::Circle`], obliquely → an [`Curve::Ellipse`], an
/// axis-parallel cut → the matching generator [`Curve::Line`], …), via
/// [`Surface::plane_section`]. So where a boolean CUTS a curved surface, the seam carries
/// exact geometry (and exports as a CIRCLE/ELLIPSE in STEP) instead of a bare polyline.
/// SNAPPED cut seams qualify, not only surviving construction rims:
/// [`snap_seam_vertices`] has already placed their vertices on the true intersection, so
/// the on-curve endpoint test below accepts them. A seam whose snap was rejected (the
/// planarity contract) keeps chord vertices and is honestly left untagged, and a
/// quadric∩quadric seam — e.g. cylinder∪cylinder — has no conic closed form at all.
///
/// The faces' operand surfaces are read from the result face tags, which [`recover_faces`]
/// sets from the surfaces carried on the fragments themselves — exact even in chained
/// booleans (a name-keyed lookup collided between an operand-that-is-a-boolean's carried
/// names and the other operand's names). An edge is tagged only with the UNIQUE section
/// curve through both its endpoints; zero matches (a legacy chord-bound seam, a skipped
/// snap) or several (tangent degeneracies) leave it untagged — never a guess.
fn attach_seam_curves(solid: &mut Solid) {
	let orig = |f: crate::topo::FaceId| -> Option<Surface> { Some(solid.face(f).surface) };
	let mut updates: Vec<(crate::topo::EdgeId, Curve)> = Vec::new();
	for e in solid.edges() {
		let he = solid.edge(e).half_edge;
		let twin = match solid.half_edge(he).twin {
			Some(t) => t,
			None => continue,
		};
		let (sa, sb) = match (orig(solid.half_edge(he).face), orig(solid.half_edge(twin).face)) {
			(Some(a), Some(b)) => (a, b),
			_ => continue,
		};
		let (po, pn, curved) = match (sa, sb) {
			(Surface::Plane { origin, normal }, c) if !matches!(c, Surface::Plane { .. }) => (origin, normal, c),
			(c, Surface::Plane { origin, normal }) if !matches!(c, Surface::Plane { .. }) => (origin, normal, c),
			_ => continue,
		};
		let sections = curved.plane_section(po, pn);
		// Both endpoints must lie on the SAME section curve, and on exactly one of
		// them, so an unrelated plane/curved adjacency — or an ambiguous tangent
		// section — is never mis-tagged.
		let va = solid.position(solid.half_edge(he).origin);
		let vb = solid.position(solid.half_edge(solid.half_edge(he).next).origin);
		let mut matching = sections.iter().filter(|c| point_on_curve(c, va) && point_on_curve(c, vb));
		if let (Some(c), None) = (matching.next(), matching.next()) {
			updates.push((e, *c));
		}
	}
	for (e, c) in updates {
		solid.set_edge_curve_by_id(e, c);
	}
}

/// Whether `p` lies on `curve` within a small tolerance.
fn point_on_curve(curve: &Curve, p: DVec3) -> bool {
	match *curve {
		Curve::Circle { center, normal, radius } => {
			let d = p - center;
			let h = d.dot(normal);
			let radial = (d - normal * h).length();
			h.abs() < 1e-6 && (radial - radius).abs() < 1e-6
		}
		Curve::Ellipse { center, normal, u, a, b } => {
			// In the ellipse plane AND satisfying x²/a² + y²/b² = 1 (not merely coplanar).
			let d = p - center;
			if d.dot(normal).abs() >= 1e-6 || a <= 0.0 || b <= 0.0 {
				return false;
			}
			let v = normal.cross(u).normalize_or_zero();
			let (x, y) = (d.dot(u) / a, d.dot(v) / b);
			(x * x + y * y - 1.0).abs() < 1e-6
		}
		Curve::Line { origin, dir } => (p - origin).cross(dir.normalize_or_zero()).length() < 1e-6,
		_ => true,
	}
}

/// Midpoint of the combined bounding box of every triangle vertex in `a` and `b`.
fn tris_bbox_center(a: &[Tri], b: &[Tri]) -> DVec3 {
	let mut lo = DVec3::splat(f64::INFINITY);
	let mut hi = DVec3::splat(f64::NEG_INFINITY);
	for t in a.iter().chain(b) {
		for v in t.v {
			lo = lo.min(v);
			hi = hi.max(v);
		}
	}
	(lo + hi) * 0.5
}

/// Shift every triangle vertex by `off` (normals are translation-invariant).
fn translate_tris(tris: &mut [Tri], off: DVec3) {
	for t in tris.iter_mut() {
		for v in t.v.iter_mut() {
			*v += off;
		}
	}
}

#[cfg(test)]
mod tests;
