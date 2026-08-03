// Copyright (c) LMCAD. Licensed under the MIT License.

//! Pre-flight **boolean hazard linter** — name the degeneracy BEFORE the op.
//!
//! The arrangement is fuzz-hardened, but three input patterns still sit in its
//! least-margin corner (DESIGN_GUIDE §7.4; learned the hard way by the RESPOOL
//! campaign, 2026-07-28): (1) face pairs that are *nearly* — not exactly —
//! coincident (sub-0.1 mm slivers between parallel planes or co-axial equal
//! cylinders), (2) exactly coincident face pairs (supported by the
//! cancel-coincident path, but margin-reducing and worth knowing about), and
//! (3) a straight edge of one operand lying INSIDE a planar face of the other
//! (a cutter side-plane on a revolve's facet-boundary meridian, or a cutter
//! bottom edge inside a coplanar-overlap region — both observed to flip
//! results invalid or crack the default tessellation depending on facet
//! phase). [`boolean_hazards`] detects all three between two operands in a
//! few hundred milliseconds, so an authoring loop (or an AI driving the
//! kernel) gets a *named* hazard with a location instead of a blind bisect
//! three ops later.
//!
//! A fourth pattern joined the list from the keyed-pulley repro
//! (`tests/keyed_pulley_acceptance.rs`, docs/FRICTION.md open frontier): a
//! **planar face lying tangent to a cylindrical wall** (a keyway slot whose
//! inner face starts exactly at the bore radius). The planar arrangement
//! cannot resolve the coincident/tangent contact — `try_*` refuses it — and
//! unlike coincident *planes* there is no supported cancel path, so the §7.7
//! remedy is: embed the face ≥ 0.1 into the wall or coincide exactly with a
//! *face*, never kiss a curved wall. [`boolean_hazards`] now detects the
//! whole kiss band (`|axis-to-plane distance − radius| ≤ tol`) pre-flight as
//! [`HazardKind::TangentPlaneOnCylinder`], with the measured tangency gap in
//! [`Hazard::separation`] and the remedy on [`HazardKind::remedy`] /
//! [`Hazard`]'s `Display`.
//!
//! Scope, stated honestly: plane/cylinder surfaces and STRAIGHT edges only
//! (curved edges are skipped — every failure observed so far involved a
//! straight edge or a plane/cylinder pair; the tangent-face class is likewise
//! detected against cylinders only, not spheres/cones/tori); hazards are
//! grouped per analytic surface pair (facet noise collapses to one entry with
//! a `count`); this is a *linter*, not a proof — an empty report raises
//! confidence but the checked booleans (`try_*`, `try_*_sealed`) remain the
//! gate.

use kernel_core::math::DVec3;

use crate::geom::Surface;
use crate::topo::{FaceId, Solid};

/// What kind of hazard a face/edge pair forms. `Coincident*` = exact within
/// `1e-7` (the supported cancel path — informational); `NearCoincident*` =
/// separated by more than exact but ≤ `tol` (the dangerous sliver band);
/// `EdgeInFace` = a straight edge of one operand lying inside (not on the
/// boundary of) a planar face of the other; `TangentPlaneOnCylinder` = a
/// planar face of one operand kissing a cylindrical wall of the other (plane
/// parallel to the axis with `|axis distance − radius| ≤ tol` — the
/// keyway-tangent-to-the-bore degeneracy, refused by the arrangement even at
/// EXACT tangency, so the whole band is a fix-me, never informational).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum HazardKind {
	CoincidentPlanes,
	NearCoincidentPlanes,
	CoincidentCylinders,
	NearCoincidentCylinders,
	EdgeInFace,
	TangentPlaneOnCylinder,
}

impl HazardKind {
	/// The DESIGN_GUIDE §7.7 remedy for this hazard class, phrased as the edit
	/// to make — `None` for the purely informational classes (exact coincidence
	/// is the *supported* cancel path). Machine callers can surface this
	/// verbatim; [`Hazard`]'s `Display` appends it automatically.
	pub fn remedy(self) -> Option<&'static str> {
		match self {
			HazardKind::CoincidentPlanes | HazardKind::CoincidentCylinders => None,
			HazardKind::NearCoincidentPlanes | HazardKind::NearCoincidentCylinders => {
				Some("embed >= 0.1 or coincide exactly, never the sliver between (DESIGN_GUIDE 7.7)")
			}
			HazardKind::EdgeInFace => {
				Some("extend the cutter until every face is fully in air or fully in material, and keep side planes off facet meridians (DESIGN_GUIDE 7.7)")
			}
			HazardKind::TangentPlaneOnCylinder => {
				Some("embed the face >= 0.1 into the curved wall or coincide exactly with a face, never kiss a curved wall (DESIGN_GUIDE 7.7)")
			}
		}
	}
}

/// One grouped hazard between operand `A` and operand `B` of an upcoming
/// boolean. `face_a`/`face_b` are exemplar faces (the first pair seen for
/// this analytic-surface pair); `count` is how many face (or edge) pairs
/// collapsed into this entry; `separation` is the worst (smallest) gap seen;
/// `at` is a representative location in model space.
#[derive(Clone, Copy, Debug)]
pub struct Hazard {
	pub kind: HazardKind,
	/// Exemplar face of operand A (for `EdgeInFace` with the edge on B, the
	/// planar face the edge lies in; for the edge on A, the face is on B and
	/// this holds the *edge-adjacent* face of A).
	pub face_a: FaceId,
	/// Exemplar face of operand B (see `face_a`).
	pub face_b: FaceId,
	/// `true` when an `EdgeInFace` hazard's edge belongs to operand A.
	pub edge_on_a: bool,
	pub separation: f64,
	pub at: DVec3,
	pub count: usize,
}

impl std::fmt::Display for Hazard {
	fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
		write!(
			f,
			"{:?} ×{} sep {:.4} at ({:.2}, {:.2}, {:.2})",
			self.kind, self.count, self.separation, self.at.x, self.at.y, self.at.z
		)?;
		if let Some(remedy) = self.kind.remedy() {
			write!(f, " — remedy: {remedy}")?;
		}
		Ok(())
	}
}

use crate::tol::{COINCIDENT_EXACT_EPS as EXACT, SURF_KEY_QUANTUM};

fn face_aabb(s: &Solid, f: FaceId) -> (DVec3, DVec3) {
	let mut lo = DVec3::splat(f64::INFINITY);
	let mut hi = DVec3::splat(f64::NEG_INFINITY);
	for v in s.face_vertices(f) {
		let p = s.position(v);
		lo = lo.min(p);
		hi = hi.max(p);
	}
	(lo, hi)
}

fn aabb_overlap(a: &(DVec3, DVec3), b: &(DVec3, DVec3), tol: f64) -> bool {
	a.0.x - tol <= b.1.x
		&& b.0.x - tol <= a.1.x
		&& a.0.y - tol <= b.1.y
		&& b.0.y - tol <= a.1.y
		&& a.0.z - tol <= b.1.z
		&& b.0.z - tol <= a.1.z
}

/// Quantized identity of an analytic surface, used to group facet noise.
fn surf_key(s: &Surface) -> (u8, [i64; 7]) {
	let q = |x: f64| (x / SURF_KEY_QUANTUM).round() as i64;
	match *s {
		Surface::Plane { origin, normal } => {
			// canonicalize sign so n and −n group together
			let flip = if normal.z < 0.0 || (normal.z == 0.0 && normal.y < 0.0) || (normal.z == 0.0 && normal.y == 0.0 && normal.x < 0.0)
			{
				-1.0
			} else {
				1.0
			};
			let n = normal * flip;
			(0, [q(n.x), q(n.y), q(n.z), q(n.dot(origin) * flip), 0, 0, 0])
		}
		Surface::Cylinder { origin, axis, radius } => {
			let flip = if axis.z < 0.0 || (axis.z == 0.0 && axis.y < 0.0) || (axis.z == 0.0 && axis.y == 0.0 && axis.x < 0.0) {
				-1.0
			} else {
				1.0
			};
			let ax = axis * flip;
			let foot = origin - ax * origin.dot(ax);
			(1, [q(ax.x), q(ax.y), q(ax.z), q(foot.x), q(foot.y), q(foot.z), q(radius)])
		}
		Surface::Sphere { center, radius } => (2, [q(center.x), q(center.y), q(center.z), q(radius), 0, 0, 0]),
		Surface::Cone { apex, axis, half_angle } => {
			(3, [q(apex.x), q(apex.y), q(apex.z), q(axis.x), q(axis.y), q(axis.z), q(half_angle)])
		}
		Surface::Torus { center, axis, major, minor } => {
			(4, [q(center.x), q(center.y), q(center.z), q(axis.x), q(axis.y), q(axis.z), q(major + 1e4 * minor)])
		}
	}
}

/// Project `p` into face `f`'s plane basis and test containment in the face
/// region (inside the outer loop, outside every inner loop). Plane faces only.
fn point_in_plane_face(s: &Solid, f: FaceId, n: DVec3, p: DVec3) -> bool {
	let helper = if n.x.abs() < 0.9 { DVec3::X } else { DVec3::Y };
	let u = n.cross(helper).normalize();
	let v = n.cross(u);
	let uv = |q: DVec3| (q.dot(u), q.dot(v));
	let (px, py) = uv(p);
	let inside_loop = |lp: crate::topo::LoopId| -> bool {
		let hes = s.loop_half_edges(lp);
		let mut inside = false;
		let pts: Vec<(f64, f64)> = hes.iter().map(|&he| uv(s.position(s.half_edge(he).origin))).collect();
		for i in 0..pts.len() {
			let (x0, y0) = pts[i];
			let (x1, y1) = pts[(i + 1) % pts.len()];
			if (y0 > py) != (y1 > py) {
				let xi = x0 + (py - y0) / (y1 - y0) * (x1 - x0);
				if xi > px {
					inside = !inside;
				}
			}
		}
		inside
	};
	let face = s.face(f);
	if !inside_loop(face.outer) {
		return false;
	}
	for &il in &face.inner {
		if inside_loop(il) {
			return false;
		}
	}
	true
}

/// Scan the straight edges of `edges_of` against the planar faces of `faces_of`
/// and group the hits. `edge_on_a` labels which operand owns the edges.
fn edge_in_face_hazards(
	edges_of: &Solid,
	faces_of: &Solid,
	edge_on_a: bool,
	tol: f64,
	out: &mut std::collections::HashMap<(u8, [i64; 7], bool), Hazard>,
) {
	// planar faces of `faces_of` with their AABBs
	let planes: Vec<(FaceId, DVec3, DVec3, (DVec3, DVec3))> = faces_of
		.faces()
		.filter_map(|f| match faces_of.face(f).surface {
			Surface::Plane { origin, normal } => Some((f, origin, normal, face_aabb(faces_of, f))),
			_ => None,
		})
		.collect();
	for e in edges_of.edges() {
		let ed = edges_of.edge(e);
		if ed.curve.is_some() {
			continue; // straight edges only (stated in the module doc)
		}
		let he = edges_of.half_edge(ed.half_edge);
		let p0 = edges_of.position(he.origin);
		let p1 = edges_of.position(edges_of.half_edge(he.next).origin);
		let elo = p0.min(p1);
		let ehi = p0.max(p1);
		if (p1 - p0).length_squared() < 1e-12 {
			continue;
		}
		let adj_face = he.face;
		for (f, origin, normal, aabb) in &planes {
			if !aabb_overlap(&(elo, ehi), aabb, tol) {
				continue;
			}
			let d0 = (p0 - *origin).dot(*normal);
			let d1 = (p1 - *origin).dot(*normal);
			if d0.abs() > tol || d1.abs() > tol {
				continue;
			}
			// the 2D basis test drops the normal component, so `mid` needs no
			// explicit projection into the plane
			let mid = (p0 + p1) * 0.5;
			if !point_in_plane_face(faces_of, *f, *normal, mid) {
				continue;
			}
			let sep = d0.abs().max(d1.abs());
			let key = (surf_key(&Surface::Plane { origin: *origin, normal: *normal }).0, surf_key(&Surface::Plane { origin: *origin, normal: *normal }).1, edge_on_a);
			let (fa, fb) = if edge_on_a { (adj_face, *f) } else { (*f, adj_face) };
			out.entry(key)
				.and_modify(|h| {
					h.count += 1;
					if sep < h.separation {
						h.separation = sep;
						h.at = mid;
					}
				})
				.or_insert(Hazard { kind: HazardKind::EdgeInFace, face_a: fa, face_b: fb, edge_on_a, separation: sep, at: mid, count: 1 });
		}
	}
}

/// Tangency test of one planar face against one cylindrical face: fires when
/// the plane is parallel to the cylinder axis and the axis-to-plane distance
/// is within `tol` of the radius — the face merely *kisses* the curved wall
/// (from inside or outside), the degeneracy the keyed-pulley repro pins.
/// `sep` is the measured tangency gap `|axis distance − radius|` (0 = exact
/// tangency); `at` is a point of the would-be tangency line clamped near the
/// overlap of the two face AABBs. Returns `None` for transversal (properly
/// embedded) or clear placements.
fn tangent_plane_cylinder(
	plane: (DVec3, DVec3),
	cylinder: (DVec3, DVec3, f64),
	aabb_a: &(DVec3, DVec3),
	aabb_b: &(DVec3, DVec3),
	tol: f64,
) -> Option<(HazardKind, f64, DVec3)> {
	let (plane_origin, plane_normal) = plane;
	let (cyl_origin, cyl_axis, radius) = cylinder;
	let n = plane_normal.normalize_or_zero();
	let axis = cyl_axis.normalize_or_zero();
	if n == DVec3::ZERO || axis == DVec3::ZERO || n.dot(axis).abs() > 1e-6 {
		return None; // plane not parallel to the axis: a transversal cut, not a kiss
	}
	let dist = (cyl_origin - plane_origin).dot(n); // signed axis-to-plane distance
	let gap = (dist.abs() - radius).abs();
	if gap > tol {
		return None;
	}
	// Tangency line: the cylinder ruling nearest the plane. Anchor it at the
	// centre of the two faces' AABB overlap so `at` lands on the actual contact.
	let toward_plane = if dist > 0.0 { -n } else { n };
	let q = cyl_origin + toward_plane * radius;
	let c = (aabb_a.0.max(aabb_b.0) + aabb_a.1.min(aabb_b.1)) * 0.5;
	let at = q + axis * (c - q).dot(axis);
	Some((HazardKind::TangentPlaneOnCylinder, gap, at))
}

/// Pre-flight lint of an upcoming boolean between `a` and `b`: report exactly
/// where the two operands present coincident / nearly-coincident analytic
/// surfaces, a planar face kissing a cylindrical wall, or an edge lying
/// inside the other's planar face — the input patterns that sit in the
/// arrangement's least-margin corner. `tol` is the
/// hazard band in model units (0.05 mm is a good authoring default: real
/// design clearances live above it, slivers live below).
///
/// Reading the report: `NearCoincident*`, `EdgeInFace` and
/// `TangentPlaneOnCylinder` are the ones to fix (add ≥0.1 embedment, move the
/// cutter face into open air, or re-phase it off the facet grid — each kind's
/// [`HazardKind::remedy`] states its §7.7 edit; note the tangent-on-cylinder
/// class has NO exact-coincidence escape, even a 0.0 gap is refused);
/// `Coincident*` (exact) is the *supported*
/// cancel-coincident path — informational, but each one is margin you could
/// reclaim with an embedment. An empty report is strong (not absolute)
/// evidence the op is well-conditioned; `try_*` / `try_*_sealed` remain the
/// actual gate.
pub fn boolean_hazards(a: &Solid, b: &Solid, tol: f64) -> Vec<Hazard> {
	let mut grouped: std::collections::HashMap<(u8, [i64; 7], bool), Hazard> = std::collections::HashMap::new();

	// ---- face-pair hazards (plane–plane, cylinder–cylinder) --------------------
	let faces_of = |s: &Solid| -> Vec<(FaceId, Surface, (DVec3, DVec3))> {
		s.faces().map(|f| (f, s.face(f).surface, face_aabb(s, f))).collect()
	};
	let fa = faces_of(a);
	let fb = faces_of(b);
	for (ia, sa, ba) in &fa {
		for (ib, sb, bb) in &fb {
			if !aabb_overlap(ba, bb, tol) {
				continue;
			}
			let hit: Option<(HazardKind, f64, DVec3)> = match (sa, sb) {
				(Surface::Plane { origin: oa, normal: na }, Surface::Plane { origin: ob, normal: nb }) => {
					if na.cross(*nb).length() < 1e-6 {
						let sep = ((*ob - *oa).dot(*na)).abs();
						if sep <= tol {
							let kind = if sep <= EXACT { HazardKind::CoincidentPlanes } else { HazardKind::NearCoincidentPlanes };
							Some((kind, sep, (ba.0 + ba.1) * 0.5))
						} else {
							None
						}
					} else {
						None
					}
				}
				(
					Surface::Cylinder { origin: oa, axis: xa, radius: ra },
					Surface::Cylinder { origin: ob, axis: xb, radius: rb },
				) => {
					if xa.cross(*xb).length() < 1e-6 {
						let d = *ob - *oa;
						let axial_off = (d - *xa * d.dot(*xa)).length();
						let sep = axial_off.max((ra - rb).abs());
						if sep <= tol {
							let kind = if sep <= EXACT {
								HazardKind::CoincidentCylinders
							} else {
								HazardKind::NearCoincidentCylinders
							};
							Some((kind, sep, (ba.0 + ba.1) * 0.5))
						} else {
							None
						}
					} else {
						None
					}
				}
				// The keyed-pulley class: a planar face kissing a cylindrical
				// wall. Either operand may hold the plane; the geometry test is
				// symmetric (plane parallel to the axis, axis-to-plane distance
				// within tol of the radius = the tangency kiss band).
				(Surface::Plane { origin: op, normal: n }, Surface::Cylinder { origin: oc, axis, radius }) => {
					tangent_plane_cylinder((*op, *n), (*oc, *axis, *radius), ba, bb, tol)
				}
				(Surface::Cylinder { origin: oc, axis, radius }, Surface::Plane { origin: op, normal: n }) => {
					tangent_plane_cylinder((*op, *n), (*oc, *axis, *radius), ba, bb, tol)
				}
				_ => None,
			};
			if let Some((kind, sep, at)) = hit {
				let ka = surf_key(sa);
				let kb = surf_key(sb);
				// one entry per unordered analytic pair: fold B's key into A's
				let mut key = ka.1;
				for (k, v) in key.iter_mut().zip(kb.1.iter()) {
					*k = k.wrapping_mul(31).wrapping_add(*v);
				}
				grouped
					.entry((ka.0.wrapping_add(kb.0).wrapping_add(10), key, false))
					.and_modify(|h| {
						h.count += 1;
						if sep < h.separation {
							h.separation = sep;
							h.at = at;
						}
					})
					.or_insert(Hazard { kind, face_a: *ia, face_b: *ib, edge_on_a: false, separation: sep, at, count: 1 });
			}
		}
	}

	// ---- edge-in-face hazards, both directions ----------------------------------
	edge_in_face_hazards(a, b, true, tol, &mut grouped);
	edge_in_face_hazards(b, a, false, tol, &mut grouped);

	let mut out: Vec<Hazard> = grouped.into_values().collect();
	out.sort_by(|x, y| x.separation.partial_cmp(&y.separation).unwrap_or(std::cmp::Ordering::Equal));
	out
}
