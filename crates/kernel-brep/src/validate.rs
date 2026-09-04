// Copyright (c) LMCAD. Licensed under the MIT License.

//! Validity oracles: half-edge invariants, the Euler–Poincaré relation, and
//! exact area / volume (the spec's correctness oracle, free from analytic faces).

use kernel_core::math::{DMat3, DVec3};
use kernel_core::{DraftReport, MassProperties, OverhangReport, SectionProperties, ThicknessOptions, ThicknessReport};

use crate::geom::{Curve, Surface};
use crate::tessellate::tessellate_default;
use crate::topo::{EdgeId, HalfEdgeId, Solid, VertexId};

/// A summary of a solid's topological health.
#[derive(Clone, Copy, Debug)]
pub struct Validity {
	/// Every half-edge has a twin (a closed boundary, no open edges).
	pub closed: bool,
	/// Every edge is used by exactly two half-edges and `next`/`prev` are consistent.
	pub manifold: bool,
	/// Euler characteristic `χ = V − E + F`.
	pub euler_characteristic: i64,
	/// Genus `G`, from `χ = 2(S − G)`.
	pub genus: i64,
	pub shells: usize,
}

impl Validity {
	/// A well-formed closed orientable solid.
	///
	/// Beyond the local half-edge invariants ([`Self::closed`], [`Self::manifold`]) this
	/// also rejects a **negative genus**: a connected closed orientable surface has genus
	/// ≥ 0 (`χ = 2(S − G) ≤ 2S`), so `G < 0` is topologically impossible and exposes a
	/// hidden self-touching / pinched arrangement that the per-edge and per-vertex checks
	/// pass but that is not a real manifold.
	pub fn is_valid(&self) -> bool {
		self.closed && self.manifold && self.genus >= 0
	}
}

/// `χ = V − E + F`.
pub fn euler_characteristic(s: &Solid) -> i64 {
	s.vertex_count() as i64 - s.edge_count() as i64 + s.face_count() as i64
}

/// Run all half-edge invariant checks and compute χ and genus.
pub fn validate(s: &Solid) -> Validity {
	let hec = s.half_edge_count();

	let mut closed = true;
	let mut consistent = true;
	let mut edge_uses = vec![0u32; s.edge_count()];
	for i in 0..hec as u32 {
		let id = HalfEdgeId(i);
		let he = *s.half_edge(id);
		if he.twin.is_none() {
			closed = false;
		}
		if s.half_edge(he.next).prev != id || s.half_edge(he.prev).next != id {
			consistent = false;
		}
		let EdgeId(e) = he.edge;
		if (e as usize) < edge_uses.len() {
			edge_uses[e as usize] += 1;
		} else {
			consistent = false;
		}
	}
	let edges_ok = edge_uses.iter().all(|&c| c == 2);

	// Loops must close (walking `next` returns to the loop start).
	let mut loops_close = true;
	for f in s.faces() {
		let outer = s.face(f).outer;
		let start = s.loop_(outer).first;
		let mut he = s.half_edge(start).next;
		let mut steps = 0;
		while he != start {
			he = s.half_edge(he).next;
			steps += 1;
			if steps > hec {
				loops_close = false;
				break;
			}
		}
	}

	// Vertex umbrellas: the outgoing half-edges of each vertex must form a single
	// rotation fan (`rotate = twin(prev(he))`). Otherwise the vertex is pinched
	// (a bowtie / two cones apex-to-apex) — 2-manifold along every edge yet not
	// a manifold at the vertex.
	let mut out_count = vec![0u32; s.vertex_count()];
	for i in 0..hec as u32 {
		out_count[s.half_edge(HalfEdgeId(i)).origin.0 as usize] += 1;
	}
	let mut vertices_manifold = true;
	for v in 0..s.vertex_count() as u32 {
		let total = out_count[v as usize];
		if total == 0 {
			continue;
		}
		let start = s.vertex(VertexId(v)).half_edge;
		let mut he = start;
		let mut len = 0u32;
		loop {
			len += 1;
			let prev = s.half_edge(he).prev;
			match s.half_edge(prev).twin {
				Some(tw) => he = tw,
				None => {
					vertices_manifold = false;
					break;
				}
			}
			if he == start || len > total {
				break;
			}
		}
		if len != total {
			vertices_manifold = false;
		}
	}

	// Shell count = connected components over face adjacency through twins
	// (`from_faces` stores a single Shell regardless of connectivity).
	let nf = s.face_count();
	let mut parent: Vec<u32> = (0..nf as u32).collect();
	fn find(p: &mut [u32], mut x: u32) -> u32 {
		while p[x as usize] != x {
			p[x as usize] = p[p[x as usize] as usize];
			x = p[x as usize];
		}
		x
	}
	for i in 0..hec as u32 {
		let he = *s.half_edge(HalfEdgeId(i));
		if let Some(tw) = he.twin {
			let ra = find(&mut parent, he.face.0);
			let rb = find(&mut parent, s.half_edge(tw).face.0);
			if ra != rb {
				parent[ra as usize] = rb;
			}
		}
	}
	let shells = (0..nf as u32).filter(|&f| find(&mut parent, f) == f).count();

	// Euler–Poincaré with ring loops: a face with an inner (hole) loop contributes an extra
	// boundary that `V − E + F` over-counts, so the boundary surface's true characteristic is
	// `V − E + F − R` (R = total inner loops). Without this a washer (a solid torus, one
	// through-hole) reads χ=2/genus 0 instead of χ=0/genus 1. R = 0 for all single-loop solids,
	// so this is a no-op for everything the primitives and booleans build today.
	let ring_loops: i64 = s.faces().map(|f| s.face(f).inner.len() as i64).sum();
	let chi = euler_characteristic(s) - ring_loops;
	Validity {
		closed,
		manifold: consistent && edges_ok && loops_close && vertices_manifold,
		euler_characteristic: chi,
		genus: shells as i64 - chi / 2,
		shells,
	}
}

/// Signed volume (mm³), from a default tessellation. Exact for planar-faced
/// solids; converges to the closed form for faceted curved solids.
pub fn volume(s: &Solid) -> f64 {
	tessellate_default(s).signed_volume()
}

/// Surface area (mm²), from a default tessellation.
pub fn area(s: &Solid) -> f64 {
	tessellate_default(s).surface_area()
}

/// **Exact** analytic volume (mm³): the f64 faceted-boundary volume plus a closed-form
/// "bulge" correction for every analytic curved face — the material between the face's
/// chord facets and the true surface. Cylinder: `½r²(Δθ − sin Δθ)·h`; sphere:
/// `Ω·r³/3 − pyramid` (Ω = facet solid angle); cone: `(tan²α/6)(Δθ − sin Δθ)(t₁³ − t₀³)`.
/// Each is sign-aware (added for a convex boss, subtracted for a concave hole/pocket), so
/// this is machine-exact for solids whose curved faces are the analytic primitives —
/// cylinder, sphere, cone — plus planar faces, including booleans with holes/pockets/
/// tubes, where the tessellation-based [`volume`] under- or over-fills by ~1–2%. Torus
/// faces get the divergence-theorem patch correction [`torus_bulge`] — exact for θ-closed
/// bands such as rim fillets (and whole tori); only a torus face spanning a partial ring
/// keeps a documented in-θ lateral residual. General freeform surfaces contribute their
/// faceted (tessellation-level) value.
///
/// **Loop-aware**: a multi-loop face (a cap with hole rings, e.g. from
/// [`crate::build::extrude_with_holes`]) contributes every loop's fan volume as wound —
/// inner loops are wound opposite the outer, so they subtract their hole's flux exactly.
/// Caveat, stated honestly: the curvature *bulge* corrections below span only the face's
/// OUTER loop; a **curved** face carrying an inner loop would have the hole's missing
/// bulge ignored. No constructor or boolean emits such a face today (booleans on
/// inner-loop faces are open bug R2), so this is unreachable, not silently wrong.
pub fn exact_volume(s: &Solid) -> f64 {
	let mut v = 0.0;
	for f in s.faces() {
		let face = s.face(f);
		let poly = s.loop_polygon(face.outer);
		// f64 faceted contribution of this face: outer loop plus every inner hole loop,
		// each AS WOUND. An inner loop is wound OPPOSITE the outer (see `FaceLoops`), so
		// its fan volume already carries the minus sign that removes the hole's flux —
		// adding it subtracts the hole. (It was subtracted here once, which flipped every
		// hole's sign and over-counted each multi-loop face by twice the hole's flux: a
		// 7-hole flange extrusion read +7.1%.)
		v += polygon_tetra_volume(&poly);
		for &inner in &face.inner {
			v += polygon_tetra_volume(&s.loop_polygon(inner));
		}
		// Analytic curvature correction for an analytic curved face.
		v += match face.surface {
			Surface::Cylinder { origin, axis, radius } => cylinder_bulge(&poly, origin, axis.normalize_or_zero(), radius),
			Surface::Sphere { center, radius } => sphere_bulge(&poly, center, radius),
			Surface::Cone { apex, axis, half_angle } => cone_bulge(&poly, apex, axis.normalize_or_zero(), half_angle),
			Surface::Torus { center, axis, major, minor } => torus_bulge(&poly, center, axis.normalize_or_zero(), major, minor),
			Surface::Plane { .. } => 0.0,
		};
	}
	v
}

/// Signed volume contribution of a planar polygon via fan tetrahedra from the world
/// origin: `Σ (a·(b×c))/6`. Exact in f64 for a flat, correctly-wound boundary loop.
fn polygon_tetra_volume(poly: &[DVec3]) -> f64 {
	let mut v = 0.0;
	for i in 1..poly.len().saturating_sub(1) {
		v += poly[0].dot(poly[i].cross(poly[i + 1])) / 6.0;
	}
	v
}

/// Two orthonormal vectors perpendicular to `axis` (assumed unit).
fn perp_basis(axis: DVec3) -> (DVec3, DVec3) {
	let t = if axis.x.abs() < 0.9 { DVec3::X } else { DVec3::Y };
	let e1 = (t - axis * t.dot(axis)).normalize_or_zero();
	(e1, axis.cross(e1))
}

/// Newell normal of a polygon loop (robust area-weighted face normal).
fn newell_normal(poly: &[DVec3]) -> DVec3 {
	let mut n = DVec3::ZERO;
	let m = poly.len();
	for i in 0..m {
		let (a, b) = (poly[i], poly[(i + 1) % m]);
		n.x += (a.y - b.y) * (a.z + b.z);
		n.y += (a.z - b.z) * (a.x + b.x);
		n.z += (a.x - b.x) * (a.y + b.y);
	}
	n.normalize_or_zero()
}

/// Volume of the circular-segment "bulge" between a cylindrical face's chord facets and
/// its true arc: `½ r² (Δθ − sin Δθ) · h`, signed `+` when the face is convex-outward
/// (a boss) and `−` when concave (a hole). `Δθ` is the face's angular span about the
/// axis, `h` its axial extent.
fn cylinder_bulge(poly: &[DVec3], origin: DVec3, axis: DVec3, radius: f64) -> f64 {
	if poly.len() < 3 || axis.length_squared() < 0.5 {
		return 0.0;
	}
	let (e1, e2) = perp_basis(axis);
	let (mut v_min, mut v_max) = (f64::INFINITY, f64::NEG_INFINITY);
	let thetas: Vec<f64> = poly
		.iter()
		.map(|&p| {
			let rel = p - origin;
			let h = rel.dot(axis);
			v_min = v_min.min(h);
			v_max = v_max.max(h);
			let radial = rel - axis * h;
			radial.dot(e2).atan2(radial.dot(e1))
		})
		.collect();
	// Angular span, unwrapped around the first vertex to avoid the ±π seam.
	let (r0, mut lo, mut hi) = (thetas[0], 0.0_f64, 0.0_f64);
	for &t in &thetas {
		let mut d = t - r0;
		while d > std::f64::consts::PI {
			d -= std::f64::consts::TAU;
		}
		while d < -std::f64::consts::PI {
			d += std::f64::consts::TAU;
		}
		lo = lo.min(d);
		hi = hi.max(d);
	}
	let d_theta = hi - lo;
	let height = (v_max - v_min).abs();
	let bulge = 0.5 * radius * radius * (d_theta - d_theta.sin()) * height;
	// Convex (outward radial normal) ⇒ add material; concave (hole) ⇒ remove it.
	let centroid = poly.iter().copied().sum::<DVec3>() / poly.len() as f64;
	let radial = {
		let rel = centroid - origin;
		(rel - axis * rel.dot(axis)).normalize_or_zero()
	};
	let sign = if newell_normal(poly).dot(radial) >= 0.0 { 1.0 } else { -1.0 };
	sign * bulge
}

/// First-moment (`∫ r dV`) contribution of the circular-segment "bulge" between a cylindrical
/// face's chord facets and its true arc, about the world origin: the segment-lens volume times
/// its centroid. The radial term `lens_volume · c = ⅔R³·h·sin³(Δθ/2)` (segment centroid distance
/// `c = 4R sin³(Δθ/2)/(3(Δθ−sinΔθ))` times the area `½R²(Δθ−sinΔθ)·h`) is written without the
/// division so it stays exact as `Δθ→0`. Signed `+`/`−` like [`cylinder_bulge`]. Added to the
/// faceted mesh's first moment this yields the EXACT moment — hence an analytic centre of mass
/// for cylindrical parts (e.g. an off-centre bore shifts the CoM by exactly the right amount).
fn cylinder_first_moment(poly: &[DVec3], origin: DVec3, axis: DVec3, radius: f64) -> DVec3 {
	if poly.len() < 3 || axis.length_squared() < 0.5 {
		return DVec3::ZERO;
	}
	let (e1, e2) = perp_basis(axis);
	let (mut v_min, mut v_max) = (f64::INFINITY, f64::NEG_INFINITY);
	let thetas: Vec<f64> = poly
		.iter()
		.map(|&p| {
			let rel = p - origin;
			let h = rel.dot(axis);
			v_min = v_min.min(h);
			v_max = v_max.max(h);
			let radial = rel - axis * h;
			radial.dot(e2).atan2(radial.dot(e1))
		})
		.collect();
	let (r0, mut lo, mut hi) = (thetas[0], 0.0_f64, 0.0_f64);
	for &t in &thetas {
		let mut d = t - r0;
		while d > std::f64::consts::PI {
			d -= std::f64::consts::TAU;
		}
		while d < -std::f64::consts::PI {
			d += std::f64::consts::TAU;
		}
		lo = lo.min(d);
		hi = hi.max(d);
	}
	let d_theta = hi - lo;
	let height = (v_max - v_min).abs();
	let lens_volume = 0.5 * radius * radius * (d_theta - d_theta.sin()) * height;
	let radial_moment = 2.0 / 3.0 * radius.powi(3) * height * (0.5 * d_theta).sin().powi(3);
	let theta_mid = r0 + 0.5 * (lo + hi);
	let bisector = e1 * theta_mid.cos() + e2 * theta_mid.sin();
	let z_mid = 0.5 * (v_min + v_max);
	// Lens moment = volume·(origin + axis·z_mid) + bisector·radial_moment.
	let moment = origin * lens_volume + axis * (lens_volume * z_mid) + bisector * radial_moment;
	let centroid = poly.iter().copied().sum::<DVec3>() / poly.len() as f64;
	let radial = {
		let rel = centroid - origin;
		(rel - axis * rel.dot(axis)).normalize_or_zero()
	};
	let sign = if newell_normal(poly).dot(radial) >= 0.0 { 1.0 } else { -1.0 };
	sign * moment
}

/// Second-moment (`∫ p pᵀ dV`) contribution of the circular-segment "bulge" between a
/// cylindrical face's chord facets and its true arc, about the **world origin** — the
/// second-order companion of [`cylinder_first_moment`], and the piece that makes the
/// INERTIA TENSOR analytic for cylindrical parts.
///
/// The lens between the facet and the arc is the 2-D circular segment (half-span
/// `α = Δθ/2` about the face's angular bisector) extruded over the face's axial range
/// `[z₀, z₁]`. In the lens's own frame (x = bisector, y = in-ring tangent, z = axis,
/// origin on the cylinder axis) the segment's closed-form planar moments are
///
/// - area `A = ½R²(Δθ − sin Δθ)`, first moment `Sx = ⅔R³ sin³α` (Sy = Sxy = 0),
/// - `Sxx = ∫x²dA = R⁴(¼(α + sinα cosα) − ½ sinα cos³α)`,
/// - `Syy = ∫y²dA = R⁴(¼(α − sinα cosα) − ⅙ sin³α cosα)`  (sector minus chord triangle),
///
/// and the prism's 3-D moments follow by integrating 1, z, z² along the axis. The world
/// tensor is assembled from the frame vectors plus the `origin`-offset cross terms, and
/// signed `+` (convex boss) / `−` (concave bore) exactly like [`cylinder_bulge`]. Summed
/// over a full cylinder the planar terms telescope to `disk − inscribed n-gon`, so the
/// faceted mesh's `∫p pᵀ dV` plus these corrections is **machine-exact** for solids whose
/// curved faces are cylindrical patches rectangular in `(θ, z)` (primitive walls, drilled
/// bores, shortened fillet walls). Like the volume/CoM helpers it assumes that patch
/// shape; sphere, cone and torus faces have their own companions
/// ([`sphere_second_moment`], [`cone_second_moment`], [`torus_lens_moments`]).
fn cylinder_second_moment(poly: &[DVec3], origin: DVec3, axis: DVec3, radius: f64) -> DMat3 {
	if poly.len() < 3 || axis.length_squared() < 0.5 {
		return DMat3::ZERO;
	}
	// θ/z-range recovery, identical to cylinder_first_moment (kept self-contained so the
	// volume-side helpers — owned by exact_volume — stay textually untouched).
	let (e1, e2) = perp_basis(axis);
	let (mut v_min, mut v_max) = (f64::INFINITY, f64::NEG_INFINITY);
	let thetas: Vec<f64> = poly
		.iter()
		.map(|&p| {
			let rel = p - origin;
			let h = rel.dot(axis);
			v_min = v_min.min(h);
			v_max = v_max.max(h);
			let radial = rel - axis * h;
			radial.dot(e2).atan2(radial.dot(e1))
		})
		.collect();
	let (r0, mut lo, mut hi) = (thetas[0], 0.0_f64, 0.0_f64);
	for &t in &thetas {
		let mut d = t - r0;
		while d > std::f64::consts::PI {
			d -= std::f64::consts::TAU;
		}
		while d < -std::f64::consts::PI {
			d += std::f64::consts::TAU;
		}
		lo = lo.min(d);
		hi = hi.max(d);
	}
	let d_theta = hi - lo;
	let alpha = 0.5 * d_theta;
	let (sin_a, cos_a) = alpha.sin_cos();
	let r4 = radius.powi(4);
	// Planar segment moments in the bisector frame (see doc comment).
	let area = 0.5 * radius * radius * (d_theta - d_theta.sin());
	let sx = 2.0 / 3.0 * radius.powi(3) * sin_a.powi(3);
	let sxx = r4 * (0.25 * (alpha + sin_a * cos_a) - 0.5 * sin_a * cos_a.powi(3));
	let syy = r4 * (0.25 * (alpha - sin_a * cos_a) - sin_a.powi(3) * cos_a / 6.0);
	// Axial prism integrals ∫dz, ∫z dz, ∫z² dz over [z₀, z₁].
	let h = v_max - v_min;
	let iz1 = 0.5 * (v_max * v_max - v_min * v_min);
	let iz2 = (v_max.powi(3) - v_min.powi(3)) / 3.0;
	// Lens frame: x = angular bisector, y = axis × x, z = axis.
	let theta_mid = r0 + 0.5 * (lo + hi);
	let bx = e1 * theta_mid.cos() + e2 * theta_mid.sin();
	let by = axis.cross(bx);
	// Local 3-D moments of the lens prism.
	let volume = area * h;
	let (cxx, cyy, czz, cxz) = (sxx * h, syy * h, area * iz2, sx * iz1);
	let m_rel = bx * (sx * h) + axis * (area * iz1); // lens first moment about `origin`
	let outer = |u: DVec3, v: DVec3| DMat3::from_cols(u * v.x, u * v.y, u * v.z);
	let c_world = outer(origin, origin) * volume
		+ outer(origin, m_rel)
		+ outer(m_rel, origin)
		+ outer(bx, bx) * cxx
		+ outer(by, by) * cyy
		+ outer(axis, axis) * czz
		+ (outer(bx, axis) + outer(axis, bx)) * cxz;
	let centroid = poly.iter().copied().sum::<DVec3>() / poly.len() as f64;
	let radial = {
		let rel = centroid - origin;
		(rel - axis * rel.dot(axis)).normalize_or_zero()
	};
	if newell_normal(poly).dot(radial) >= 0.0 {
		c_world
	} else {
		-c_world
	}
}

/// Volume of the "bulge" between a spherical face's chord facet and the true sphere:
/// `Ω·r³/3 − V_cone`, where `Ω` is the facet's solid angle at the sphere centre (summed
/// over a fan via the Van Oosterom–Strackee formula) and `V_cone` the pyramid from the
/// centre to the facet. Signed `+` for a convex (outward) face, `−` for a spherical
/// pocket. Summed over a whole sphere this telescopes to `4/3·π·r³ − inscribed`.
fn sphere_bulge(poly: &[DVec3], center: DVec3, radius: f64) -> f64 {
	if poly.len() < 3 {
		return 0.0;
	}
	let vs: Vec<DVec3> = poly.iter().map(|&p| p - center).collect();
	let (mut omega, mut cone) = (0.0_f64, 0.0_f64);
	for i in 1..vs.len() - 1 {
		let (a, b, c) = (vs[0], vs[i], vs[i + 1]);
		let num = a.dot(b.cross(c));
		let den = a.length() * b.length() * c.length() + a.dot(b) * c.length() + a.dot(c) * b.length() + b.dot(c) * a.length();
		omega += 2.0 * num.atan2(den);
		cone += num / 6.0;
	}
	let bulge = omega.abs() * radius.powi(3) / 3.0 - cone.abs();
	let centroid = poly.iter().copied().sum::<DVec3>() / poly.len() as f64;
	let radial = (centroid - center).normalize_or_zero();
	let sign = if newell_normal(poly).dot(radial) >= 0.0 { 1.0 } else { -1.0 };
	sign * bulge
}

/// First-moment (`∫ r dV`) contribution of the "bulge" between a spherical face's chord facet
/// and the true sphere, about the world origin — the sphere analogue of [`cylinder_first_moment`].
/// The lens = spherical sector − pyramid; its first moment about the centre is `(r⁴/4)·V −
/// Σ pyramid-tetra moments`, where `V = ½∮ r×dr = ½ Σ_edges acos(ûᵢ·ûⱼ)·(ûᵢ×ûⱼ)/|ûᵢ×ûⱼ|` is the
/// spherical patch's vector solid angle (its vector area on the unit sphere). The world moment
/// adds `center · lens_volume`, signed like [`sphere_bulge`]; the patch-relative term is
/// already winding-signed (see the comment at the return). Summed over a whole sphere the
/// patch moments cancel and this telescopes to `center · (sphere − inscribed) volume`.
fn sphere_first_moment(poly: &[DVec3], center: DVec3, radius: f64) -> DVec3 {
	if poly.len() < 3 {
		return DVec3::ZERO;
	}
	let vs: Vec<DVec3> = poly.iter().map(|&p| p - center).collect();
	let n = vs.len();
	let (mut omega, mut pyr_vol, mut pyr_moment) = (0.0_f64, 0.0_f64, DVec3::ZERO);
	for i in 1..n - 1 {
		let (a, b, c) = (vs[0], vs[i], vs[i + 1]);
		let num = a.dot(b.cross(c));
		let den = a.length() * b.length() * c.length() + a.dot(b) * c.length() + a.dot(c) * b.length() + b.dot(c) * a.length();
		omega += 2.0 * num.atan2(den);
		let vol = num / 6.0;
		pyr_vol += vol;
		pyr_moment += (a + b + c) / 4.0 * vol;
	}
	// Vector solid angle (vector area of the spherical patch).
	let mut vsa = DVec3::ZERO;
	for i in 0..n {
		let a = vs[i].normalize_or_zero();
		let b = vs[(i + 1) % n].normalize_or_zero();
		let cr = a.cross(b);
		let l = cr.length();
		if l > 1e-12 {
			vsa += cr / l * a.dot(b).clamp(-1.0, 1.0).acos();
		}
	}
	vsa *= 0.5;
	let sector_moment = vsa * (radius.powi(4) / 4.0);
	let lens_moment_c = sector_moment - pyr_moment;
	let lens_vol = omega.abs() * radius.powi(3) / 3.0 - pyr_vol.abs();
	let radial = (poly.iter().copied().sum::<DVec3>() / n as f64 - center).normalize_or_zero();
	let sign = if newell_normal(poly).dot(radial) >= 0.0 { 1.0 } else { -1.0 };
	// `lens_moment_c` is built from winding-signed integrals (vector solid angle, fan
	// pyramids), and a sphere face's winding IS its convexity: wound outward on a boss
	// (where the sum already equals the unsigned lens moment) and inward on a pocket
	// (already negated). Only the |lens volume| × `center` term needs the explicit sign.
	// Applying `sign` to BOTH terms double-flipped the patch term on concave faces — the
	// ~1e-3 hemispherical-dimple CoM residual once mis-attributed to flat-cap faceting;
	// see `inertia_tensor_subtracts_a_hemispherical_dimples_lenses` in brep_validity.rs.
	center * (sign * lens_vol) + lens_moment_c
}

/// Second-moment (`∫ p pᵀ dV`) contribution of the "bulge" lens between a spherical face's
/// chord facet and the true sphere, about the **world origin** — the sphere companion of
/// [`cylinder_second_moment`], built on the same sector − pyramid split as
/// [`sphere_first_moment`]: lens = (radial sector under the patch) − (fan pyramid to the
/// facet).
///
/// The sector's second moment about the centre is `(R⁵/5)·T` with `T = ∫_patch û ûᵀ dΩ`, the
/// patch's solid-angle second moment. For a patch bounded by great-circle arcs (the radial
/// projection of the chord polygon onto the sphere) `T` has a closed form: apply
/// `∮ n xᵀ dA = V·Id` (the divergence theorem, per column) to the closed unit-radius sector,
/// whose lateral boundary is one flat circular sector per polygon edge with first area
/// moment `⅓·tan(γ/2)·(ûᵢ + ûⱼ)` and outward normal `−(ûᵢ×ûⱼ)/|ûᵢ×ûⱼ|`, giving
///
///   `T = (Ω/3)·Id + ⅓ Σ_edges (ûᵢ×ûⱼ) ⊗ (ûᵢ+ûⱼ) / (1 + ûᵢ·ûⱼ)`
///
/// — trig-free (`tan(γ/2)/|ûᵢ×ûⱼ| = 1/(1+ûᵢ·ûⱼ)`) and exact as the edge length → 0. The fan
/// pyramid's second moments are the standard tetra closed form `(V/20)(aaᵀ+bbᵀ+ccᵀ+ssᵀ)`.
/// **Every term is winding-signed**, and a sphere face's winding is its convexity (wound
/// outward on a boss, inward on a pocket), so — unlike the θ/z-range-based cylinder helper —
/// no explicit concave-sign factor is needed: a boss adds its lens, a dimple subtracts it.
/// Summed over a whole sphere the edge terms cancel pairwise (each interior arc is traversed
/// twice, oppositely) and the total telescopes to `ball − inscribed polyhedron`, so the
/// faceted mesh's `∫ p pᵀ dV` plus these corrections is **machine-exact** for any solid whose
/// spherical faces carry their vertices on the tagged sphere — full/partial spheres,
/// hemispheres, hemispherical dimples. A boolean-clipped sphere facet with vertices OFF the
/// sphere (a cut mid-quad) degrades gracefully to patch-projection accuracy, the same
/// documented limit as [`sphere_bulge`].
fn sphere_second_moment(poly: &[DVec3], center: DVec3, radius: f64) -> DMat3 {
	if poly.len() < 3 {
		return DMat3::ZERO;
	}
	let vs: Vec<DVec3> = poly.iter().map(|&p| p - center).collect();
	let n = vs.len();
	let outer = |u: DVec3, v: DVec3| DMat3::from_cols(u * v.x, u * v.y, u * v.z);
	// Winding-signed fan: solid angle Ω (Van Oosterom–Strackee) plus the facet pyramid's
	// volume and first/second moments about the centre.
	let (mut omega, mut pyr_vol) = (0.0_f64, 0.0_f64);
	let (mut pyr_m1, mut pyr_m2) = (DVec3::ZERO, DMat3::ZERO);
	for i in 1..n - 1 {
		let (a, b, c) = (vs[0], vs[i], vs[i + 1]);
		let num = a.dot(b.cross(c));
		let den = a.length() * b.length() * c.length() + a.dot(b) * c.length() + a.dot(c) * b.length() + b.dot(c) * a.length();
		omega += 2.0 * num.atan2(den);
		let vol = num / 6.0;
		pyr_vol += vol;
		let s = a + b + c;
		pyr_m1 += s / 4.0 * vol;
		pyr_m2 += (outer(a, a) + outer(b, b) + outer(c, c) + outer(s, s)) * (vol / 20.0);
	}
	// Patch solid-angle moments from the great-arc boundary: the vector solid angle ∫û dΩ
	// (exactly as in sphere_first_moment) and the boundary part of T.
	let units: Vec<DVec3> = vs.iter().map(|v| v.normalize_or_zero()).collect();
	let mut vsa = DVec3::ZERO;
	let mut t_edges = DMat3::ZERO;
	for i in 0..n {
		let (a, b) = (units[i], units[(i + 1) % n]);
		let cr = a.cross(b);
		let l = cr.length();
		if l > 1e-12 {
			vsa += cr / l * a.dot(b).clamp(-1.0, 1.0).acos();
		}
		let denom = 1.0 + a.dot(b);
		if denom > 1e-12 {
			t_edges += outer(cr, a + b) * (1.0 / (3.0 * denom));
		}
	}
	vsa *= 0.5;
	let t = DMat3::from_diagonal(DVec3::splat(omega / 3.0)) + t_edges;
	// Lens = sector − pyramid about the centre, then shift to the world origin.
	let lens_vol = omega * radius.powi(3) / 3.0 - pyr_vol;
	let lens_m1 = vsa * (radius.powi(4) / 4.0) - pyr_m1;
	let lens_m2 = t * (radius.powi(5) / 5.0) - pyr_m2;
	lens_m2 + outer(center, lens_m1) + outer(lens_m1, center) + outer(center, center) * lens_vol
}

/// Volume of the "bulge" between a conical face's chord facet and the true cone:
/// `(tan²α/6)(Δθ − sin Δθ)(t₁³ − t₀³)` — the cylinder segment integrated along the
/// cone's linearly-tapering radius `r(t) = t·tanα` between its axial extents `t₀..t₁`
/// from the apex. Signed `+` convex / `−` concave. Summed over a cone → `π·R²·h/3`.
fn cone_bulge(poly: &[DVec3], apex: DVec3, axis: DVec3, half_angle: f64) -> f64 {
	if poly.len() < 3 || axis.length_squared() < 0.5 {
		return 0.0;
	}
	let (e1, e2) = perp_basis(axis);
	let (mut t_min, mut t_max) = (f64::INFINITY, f64::NEG_INFINITY);
	let mut thetas: Vec<f64> = Vec::new();
	for &p in poly {
		let rel = p - apex;
		let t = rel.dot(axis);
		t_min = t_min.min(t);
		t_max = t_max.max(t);
		let radial = rel - axis * t;
		if radial.length() > 1e-9 {
			thetas.push(radial.dot(e2).atan2(radial.dot(e1)));
		}
	}
	if thetas.is_empty() {
		return 0.0;
	}
	let (r0, mut lo, mut hi) = (thetas[0], 0.0_f64, 0.0_f64);
	for &t in &thetas {
		let mut d = t - r0;
		while d > std::f64::consts::PI {
			d -= std::f64::consts::TAU;
		}
		while d < -std::f64::consts::PI {
			d += std::f64::consts::TAU;
		}
		lo = lo.min(d);
		hi = hi.max(d);
	}
	let d_theta = hi - lo;
	let tan_a = half_angle.tan();
	let bulge = tan_a * tan_a / 6.0 * (d_theta - d_theta.sin()) * (t_max.powi(3) - t_min.powi(3)).abs();
	let centroid = poly.iter().copied().sum::<DVec3>() / poly.len() as f64;
	let rel = centroid - apex;
	let radial = (rel - axis * rel.dot(axis)).normalize_or_zero();
	let sign = if newell_normal(poly).dot(radial) >= 0.0 { 1.0 } else { -1.0 };
	sign * bulge
}

/// First-moment (`∫ r dV`) contribution of the conical-segment "bulge" between a conical face's
/// chord facets and its true cone, about the world origin — the cone analogue of
/// [`cylinder_first_moment`]. The lens at axial distance `t` from the apex is a circular segment
/// of radius `t·tanα`; integrating its area × position along the taper gives, in the apex frame,
/// an axial moment `tan²α/8·(Δθ−sinΔθ)·(t₁⁴−t₀⁴)` and a radial moment `tan³α/6·sin³(Δθ/2)·
/// (t₁⁴−t₀⁴)` (the `Δθ−sinΔθ` cancels, exact as `Δθ→0`). Signed like [`cone_bulge`]; summed into
/// the faceted mesh first moment it yields an exact centre of mass for conical parts.
fn cone_first_moment(poly: &[DVec3], apex: DVec3, axis: DVec3, half_angle: f64) -> DVec3 {
	if poly.len() < 3 || axis.length_squared() < 0.5 {
		return DVec3::ZERO;
	}
	// Canonical frame: axis points from the apex into the cone body (t ≥ 0).
	let probe: f64 = poly.iter().map(|&p| (p - apex).dot(axis)).sum();
	let axis = if probe < 0.0 { -axis } else { axis };
	let (e1, e2) = perp_basis(axis);
	let (mut t_min, mut t_max) = (f64::INFINITY, f64::NEG_INFINITY);
	let mut thetas: Vec<f64> = Vec::new();
	for &p in poly {
		let rel = p - apex;
		let t = rel.dot(axis);
		t_min = t_min.min(t);
		t_max = t_max.max(t);
		let radial = rel - axis * t;
		if radial.length() > 1e-9 {
			thetas.push(radial.dot(e2).atan2(radial.dot(e1)));
		}
	}
	if thetas.is_empty() {
		return DVec3::ZERO;
	}
	let (r0, mut lo, mut hi) = (thetas[0], 0.0_f64, 0.0_f64);
	for &t in &thetas {
		let mut d = t - r0;
		while d > std::f64::consts::PI {
			d -= std::f64::consts::TAU;
		}
		while d < -std::f64::consts::PI {
			d += std::f64::consts::TAU;
		}
		lo = lo.min(d);
		hi = hi.max(d);
	}
	let d_theta = hi - lo;
	let tan_a = half_angle.tan();
	let dt3 = t_max.powi(3) - t_min.powi(3);
	let dt4 = t_max.powi(4) - t_min.powi(4);
	let lens_volume = tan_a * tan_a / 6.0 * (d_theta - d_theta.sin()) * dt3;
	let axial_moment = tan_a * tan_a / 8.0 * (d_theta - d_theta.sin()) * dt4;
	let radial_moment = tan_a.powi(3) / 6.0 * (0.5 * d_theta).sin().powi(3) * dt4;
	let theta_mid = r0 + 0.5 * (lo + hi);
	let bisector = e1 * theta_mid.cos() + e2 * theta_mid.sin();
	let moment = apex * lens_volume + axis * axial_moment + bisector * radial_moment;
	let centroid = poly.iter().copied().sum::<DVec3>() / poly.len() as f64;
	let rel = centroid - apex;
	let radial = (rel - axis * rel.dot(axis)).normalize_or_zero();
	let sign = if newell_normal(poly).dot(radial) >= 0.0 { 1.0 } else { -1.0 };
	sign * moment
}

/// Second-moment (`∫ p pᵀ dV`) contribution of the conical-segment "bulge" between a conical
/// face's chord facets and its true cone, about the **world origin** — the cone companion of
/// [`cylinder_second_moment`] and the second-order companion of [`cone_first_moment`].
///
/// The lens slice at axial distance `t` from the apex is the planar circular segment of
/// radius `ρ(t) = t·tanα` and half-span `β = Δθ/2` about the angular bisector, with the same
/// closed-form planar moments as the cylinder's segment (area `ρ²A₁`, `∫x dA = ρ³S₁`,
/// `∫x² dA = ρ⁴Cxx₁`, `∫y² dA = ρ⁴Cyy₁`; `x` = bisector, `y` = in-ring tangent). Because
/// `ρ ∝ t`, every prism integral along the taper reduces to a power integral `∫tᵏ dt`, and
/// all second moments in the apex frame (x = bisector, y = axis × x, z = axis) carry the
/// factor `(t₁⁵ − t₀⁵)/5`; `xy`/`yz` vanish by mirror symmetry about the bisector plane. The
/// world tensor adds the apex-offset cross terms and is signed `+` (convex cone wall) /
/// `−` (concave countersink pocket) from the face winding versus the outward radial, exactly
/// like [`cone_bulge`]. Summed over a full cone the slices telescope to `disk − inscribed
/// n-gon` at every height, so the faceted `∫ p pᵀ dV` plus these corrections is
/// **machine-exact** for solids whose conical faces are rectangular in `(θ, t)` (primitive
/// cone walls, apex-to-base triangles, countersink pockets); like the cylinder helper it
/// assumes that patch shape and degrades to chord accuracy on obliquely-clipped patches.
fn cone_second_moment(poly: &[DVec3], apex: DVec3, axis: DVec3, half_angle: f64) -> DMat3 {
	if poly.len() < 3 || axis.length_squared() < 0.5 {
		return DMat3::ZERO;
	}
	// Canonical frame (axis from the apex into the body, t ≥ 0) and θ/t-range recovery,
	// identical to cone_first_moment.
	let probe: f64 = poly.iter().map(|&p| (p - apex).dot(axis)).sum();
	let axis = if probe < 0.0 { -axis } else { axis };
	let (e1, e2) = perp_basis(axis);
	let (mut t_min, mut t_max) = (f64::INFINITY, f64::NEG_INFINITY);
	let mut thetas: Vec<f64> = Vec::new();
	for &p in poly {
		let rel = p - apex;
		let t = rel.dot(axis);
		t_min = t_min.min(t);
		t_max = t_max.max(t);
		let radial = rel - axis * t;
		if radial.length() > 1e-9 {
			thetas.push(radial.dot(e2).atan2(radial.dot(e1)));
		}
	}
	if thetas.is_empty() {
		return DMat3::ZERO;
	}
	let (r0, mut lo, mut hi) = (thetas[0], 0.0_f64, 0.0_f64);
	for &t in &thetas {
		let mut d = t - r0;
		while d > std::f64::consts::PI {
			d -= std::f64::consts::TAU;
		}
		while d < -std::f64::consts::PI {
			d += std::f64::consts::TAU;
		}
		lo = lo.min(d);
		hi = hi.max(d);
	}
	let d_theta = hi - lo;
	let beta = 0.5 * d_theta;
	let (sin_b, cos_b) = beta.sin_cos();
	// Unit-radius planar segment moments in the bisector frame (see cylinder_second_moment).
	let a1 = 0.5 * (d_theta - d_theta.sin());
	let s1 = 2.0 / 3.0 * sin_b.powi(3);
	let cxx1 = 0.25 * (beta + sin_b * cos_b) - 0.5 * sin_b * cos_b.powi(3);
	let cyy1 = 0.25 * (beta - sin_b * cos_b) - sin_b.powi(3) * cos_b / 6.0;
	// Taper integrals ∫tᵏ dt over [t₀, t₁] (ρ(t) = t·tanα turns every integrand into a power of t).
	let dt3 = (t_max.powi(3) - t_min.powi(3)) / 3.0;
	let dt4 = (t_max.powi(4) - t_min.powi(4)) / 4.0;
	let dt5 = (t_max.powi(5) - t_min.powi(5)) / 5.0;
	let tan_a = half_angle.tan();
	let (tan2, tan3, tan4) = (tan_a * tan_a, tan_a.powi(3), tan_a.powi(4));
	let volume = a1 * tan2 * dt3;
	// Apex-frame lens moments: first (bisector + axial, matching cone_first_moment) and
	// second (xx, yy, zz, xz).
	let m_rel_bx = s1 * tan3 * dt4;
	let m_rel_ax = a1 * tan2 * dt4;
	let cxx = cxx1 * tan4 * dt5;
	let cyy = cyy1 * tan4 * dt5;
	let czz = a1 * tan2 * dt5;
	let cxz = s1 * tan3 * dt5;
	let theta_mid = r0 + 0.5 * (lo + hi);
	let bx = e1 * theta_mid.cos() + e2 * theta_mid.sin();
	let by = axis.cross(bx);
	let m_rel = bx * m_rel_bx + axis * m_rel_ax;
	let outer = |u: DVec3, v: DVec3| DMat3::from_cols(u * v.x, u * v.y, u * v.z);
	let c_world = outer(apex, apex) * volume
		+ outer(apex, m_rel)
		+ outer(m_rel, apex)
		+ outer(bx, bx) * cxx
		+ outer(by, by) * cyy
		+ outer(axis, axis) * czz
		+ (outer(bx, axis) + outer(axis, bx)) * cxz;
	let centroid = poly.iter().copied().sum::<DVec3>() / poly.len() as f64;
	let rel = centroid - apex;
	let radial = (rel - axis * rel.dot(axis)).normalize_or_zero();
	if newell_normal(poly).dot(radial) >= 0.0 {
		c_world
	} else {
		-c_world
	}
}

/// Volume of the "bulge" between a toroidal face's chord facet and the true torus patch.
/// A torus is doubly curved with no ruled direction, so — unlike the cylinder/cone — there
/// is no circular-segment reduction. Instead this returns the divergence-theorem volume
/// `⅓∮ r·n dA` of the analytic patch (recovered from the face's ring-angle span `Δθ` and
/// tube-angle range `[ψ₀,ψ₁]`) minus the chord facet it replaces. With the torus normal
/// `n = e_r cosψ + axis sinψ` the centre-relative integrand `(P−C)·n = major cosψ + minor`;
/// the world-origin cross term `C·∮n dA` is kept so the correction holds for an *open* band
/// (a fillet) as well.
///
/// The patch−facet flux difference alone misses the **lateral** boundary of the lens, where
/// the patch's rim (an arc) overhangs the facet's rim (its chord). The in-θ laterals lie in
/// meridian planes through the axis and cancel pairwise around a band closed in θ; the two
/// ψ-row laterals are **horizontal circular-segment slivers** (area `½ρ²(Δθ − sin Δθ)` at
/// ring radius `ρ(ψ) = major + minor·cosψ`, axial offset `d(ψ) = C·axis + minor·sinψ`) and
/// are added here per face: `[d·σ](ψ₁) − [d·σ](ψ₀)`. Faces sharing a ψ-row cancel their
/// sliver terms, so summing over any (θ-closed) band — a rim-fillet quarter band, a full
/// torus (where the total still telescopes to `2π²·major·minor²`) — yields the **exact**
/// lens volume: the band's exposed end slivers close machine-exactly against the adjacent
/// cap plane and wall cylinder. A lone face spanning a partial ring keeps the in-θ lateral
/// residual (documented patch-projection accuracy), as before.
fn torus_bulge(poly: &[DVec3], center: DVec3, axis: DVec3, major: f64, minor: f64) -> f64 {
	if poly.len() < 3 || axis.length_squared() < 0.5 || minor <= 0.0 || major <= 0.0 {
		return 0.0;
	}
	let (e1, e2) = perp_basis(axis);
	let mut thetas: Vec<f64> = Vec::with_capacity(poly.len());
	let mut psis: Vec<f64> = Vec::with_capacity(poly.len());
	for &p in poly {
		let rel = p - center;
		let h = rel.dot(axis);
		let radial = rel - axis * h;
		thetas.push(radial.dot(e2).atan2(radial.dot(e1)));
		psis.push(h.atan2(radial.length() - major));
	}
	// Unwrap each angular range around its first vertex to avoid the ±π seam.
	let unwrap_range = |vals: &[f64]| -> (f64, f64) {
		let v0 = vals[0];
		let (mut lo, mut hi) = (0.0_f64, 0.0_f64);
		for &t in vals {
			let mut d = t - v0;
			while d > std::f64::consts::PI {
				d -= std::f64::consts::TAU;
			}
			while d < -std::f64::consts::PI {
				d += std::f64::consts::TAU;
			}
			lo = lo.min(d);
			hi = hi.max(d);
		}
		(v0 + lo, v0 + hi)
	};
	let (theta0, theta1) = unwrap_range(&thetas);
	let (psi0, psi1) = unwrap_range(&psis);
	let d_theta = theta1 - theta0;
	let d_psi = psi1 - psi0;
	let (d_sin, d_cos) = (psi1.sin() - psi0.sin(), psi1.cos() - psi0.cos());
	let d_sin2 = (2.0 * psi1).sin() - (2.0 * psi0).sin();
	let d_sinsq = psi1.sin().powi(2) - psi0.sin().powi(2);
	// ⅓∮(P−C)·n dA = ⅓·minor·Δθ·∫(major cosψ+minor)(major+minor cosψ)dψ
	let i_psi = (major * major + minor * minor) * d_sin + major * minor * (d_psi * 0.5 + d_sin2 * 0.25) + major * minor * d_psi;
	let centered_flux = minor * d_theta * i_psi;
	// World-origin cross term C·∮n dA, with ∮n dA = ∫e_r dθ·∫cosψ·minor(major+minor cosψ)dψ
	//                                           + axis·Δθ·∫sinψ·minor(major+minor cosψ)dψ.
	let er_vec = e1 * (theta1.sin() - theta0.sin()) + e2 * (theta0.cos() - theta1.cos());
	let c_psi = minor * (major * d_sin + minor * (d_psi * 0.5 + d_sin2 * 0.25));
	let s_psi = minor * (-major * d_cos + minor * 0.5 * d_sinsq);
	let patch_n = er_vec * c_psi + axis * (d_theta * s_psi);
	// Lateral ψ-row slivers of the lens (see the doc comment): horizontal circular segments
	// between the patch's rim arc and the facet's rim chord, flux `axial offset × area`.
	let sliver = |psi: f64| {
		let rho = major + minor * psi.cos();
		(center.dot(axis) + minor * psi.sin()) * 0.5 * rho * rho * (d_theta - d_theta.sin())
	};
	let lateral = sliver(psi1) - sliver(psi0);
	let true_vol = (centered_flux + center.dot(patch_n) + lateral) / 3.0;
	// Orient to the face's actual winding (canonical outward = e_r cosψ + axis sinψ).
	let centroid = poly.iter().copied().sum::<DVec3>() / poly.len() as f64;
	let rel = centroid - center;
	let hc = rel.dot(axis);
	let radial = rel - axis * hc;
	let psi_c = hc.atan2(radial.length() - major);
	let outward = radial.normalize_or_zero() * psi_c.cos() + axis * psi_c.sin();
	let s = if newell_normal(poly).dot(outward) >= 0.0 { 1.0 } else { -1.0 };
	s * true_vol - polygon_tetra_volume(poly)
}

/// First- AND second-moment (`∫ p dV`, `∫ p pᵀ dV`) contribution of the "bulge" lens
/// between a toroidal face's chord facet and the true torus patch, about the **world
/// origin** — the torus companion of [`cylinder_first_moment`]/[`cylinder_second_moment`],
/// returned as one `(first, second)` pair because both derive from a single
/// divergence-theorem pass over the same closed lens boundary (the construction of
/// [`torus_bulge`], extended to higher moments).
///
/// The lens is bounded by the analytic patch (recovered from the face's ring-angle span
/// `Δθ` and tube-angle range `[ψ₀, ψ₁]`), the two horizontal ψ-row **sliver** faces
/// (circular segments between the patch's rim arcs and the facet's rim chords), and the
/// reversed chord facet. A torus is doubly curved with no segment/sector reduction, but
/// every lens moment is still closed-form through the radial gauge `∫_V f dV =
/// ∮ f·(q·n) dA / (3 + deg f)` (valid for `f` homogeneous of degree `deg` in the
/// centre-relative position `q`): on the patch `q·n = R cosψ + r` and `dA =
/// r(R + r cosψ) dθ dψ`, so every component separates into elementary trig-polynomial
/// integrals in `θ` and `ψ`; on a ψ-row sliver `q·n = ±r sinψ` and the planar circular
/// segment has the same closed-form area/first/second moments the cylinder helper uses;
/// the facet term is the standard origin-fan tetra closed form. Signed `+` (convex,
/// wound outward) / `−` (concave pocket) from the winding against the canonical torus
/// normal, exactly like [`torus_bulge`].
///
/// Exactness scope (matching [`torus_bulge`], stated honestly): the in-θ lateral faces of
/// the lens lie in meridian planes and cancel pairwise around any **θ-closed** band —
/// adjacent facets of a full torus, a rim-fillet band — so summed over such bands the
/// faceted mesh's moments plus these corrections are **machine-exact** (verified against
/// the closed-form torus inertia `I_axis = m(R² + ¾r²)`, `I_perp = m(½R² + ⅝r²)` in
/// `tests/mass_properties_torus.rs`). A lone face spanning a partial ring keeps the
/// in-θ lateral residual (patch-projection accuracy), as before.
fn torus_lens_moments(poly: &[DVec3], center: DVec3, axis: DVec3, major: f64, minor: f64) -> (DVec3, DMat3) {
	if poly.len() < 3 || axis.length_squared() < 0.5 || minor <= 0.0 || major <= 0.0 {
		return (DVec3::ZERO, DMat3::ZERO);
	}
	let (r_maj, r_min) = (major, minor);
	let (e1, e2) = perp_basis(axis);
	// θ/ψ-range recovery, identical to torus_bulge (kept self-contained so the
	// volume-side helper — owned by exact_volume — stays textually untouched).
	let mut thetas: Vec<f64> = Vec::with_capacity(poly.len());
	let mut psis: Vec<f64> = Vec::with_capacity(poly.len());
	for &p in poly {
		let rel = p - center;
		let h = rel.dot(axis);
		let radial = rel - axis * h;
		thetas.push(radial.dot(e2).atan2(radial.dot(e1)));
		psis.push(h.atan2(radial.length() - r_maj));
	}
	let unwrap_range = |vals: &[f64]| -> (f64, f64) {
		let v0 = vals[0];
		let (mut lo, mut hi) = (0.0_f64, 0.0_f64);
		for &t in vals {
			let mut d = t - v0;
			while d > std::f64::consts::PI {
				d -= std::f64::consts::TAU;
			}
			while d < -std::f64::consts::PI {
				d += std::f64::consts::TAU;
			}
			lo = lo.min(d);
			hi = hi.max(d);
		}
		(v0 + lo, v0 + hi)
	};
	let (theta0, theta1) = unwrap_range(&thetas);
	let (psi0, psi1) = unwrap_range(&psis);
	let d_theta = theta1 - theta0;

	// θ-integrals of {1, cosθ, sinθ, cos²θ, sin²θ, cosθ·sinθ} over [θ₀, θ₁].
	let i1 = d_theta;
	let ic = theta1.sin() - theta0.sin();
	let is_ = theta0.cos() - theta1.cos();
	let icc = 0.5 * d_theta + 0.25 * ((2.0 * theta1).sin() - (2.0 * theta0).sin());
	let iss = d_theta - icc;
	let ics = 0.5 * (theta1.sin().powi(2) - theta0.sin().powi(2));

	// ψ-integrals ∫cosᵏψ dψ (k = 0..4) and ∫cosᵏψ·sinψ dψ over [ψ₀, ψ₁].
	let d_psi = psi1 - psi0;
	let (s0, s1) = (psi0.sin(), psi1.sin());
	let d_sin2 = (2.0 * psi1).sin() - (2.0 * psi0).sin();
	let icos = [
		d_psi,
		s1 - s0,
		0.5 * d_psi + 0.25 * d_sin2,
		(s1 - s1.powi(3) / 3.0) - (s0 - s0.powi(3) / 3.0),
		0.375 * d_psi + 0.25 * d_sin2 + ((4.0 * psi1).sin() - (4.0 * psi0).sin()) / 32.0,
	];
	let icos_s = |k: usize| (psi0.cos().powi(k as i32 + 1) - psi1.cos().powi(k as i32 + 1)) / (k as f64 + 1.0);
	let int_poly = |co: &[f64]| co.iter().enumerate().map(|(k, &a)| a * icos[k]).sum::<f64>();
	let int_poly_s = |co: &[f64]| co.iter().enumerate().map(|(k, &a)| a * icos_s(k)).sum::<f64>();

	// Flux weight w(ψ) = (q·n)·(area element / dθdψ) = r(R cosψ + r)(R + r cosψ) and its
	// ρ = R + r cosψ multiples, as polynomials in c = cosψ.
	let w = [r_maj * r_min * r_min, r_min * (r_maj * r_maj + r_min * r_min), r_maj * r_min * r_min];
	let conv_rho = |co: &[f64]| -> Vec<f64> {
		// multiply by ρ = R + r·c
		let mut out = vec![0.0; co.len() + 1];
		for (k, &a) in co.iter().enumerate() {
			out[k] += r_maj * a;
			out[k + 1] += r_min * a;
		}
		out
	};
	let rho_w = conv_rho(&w);
	let rho2_w = conv_rho(&rho_w);
	let iw = int_poly(&w);
	let irho_w = int_poly(&rho_w);
	let irho2_w = int_poly(&rho2_w);
	let izs_w = r_min * int_poly_s(&w); // ∫ z·w, z = r sinψ
	let irho_zs_w = r_min * int_poly_s(&rho_w);
	// ∫ z²·w = r²∫(1 − c²)·w
	let iz2_w = r_min * r_min * int_poly(&[w[0], w[1], w[2] - w[0], -w[1], -w[2]]);

	// Patch fluxes in the LOCAL frame (origin at `center`, x = e1, y = e2, z = axis):
	// q = (ρ cosθ, ρ sinθ, r sinψ), gauge divisor 3 + deg.
	let vp = i1 * iw / 3.0;
	let mp = DVec3::new(ic * irho_w, is_ * irho_w, i1 * izs_w) / 4.0;
	let (sp_xx, sp_yy, sp_xy) = (icc * irho2_w / 5.0, iss * irho2_w / 5.0, ics * irho2_w / 5.0);
	let (sp_xz, sp_yz, sp_zz) = (ic * irho_zs_w / 5.0, is_ * irho_zs_w / 5.0, i1 * iz2_w / 5.0);

	// ψ-row sliver fluxes (outward +axis at ψ₁, −axis at ψ₀): a planar circular segment
	// of radius ρ(ψ) about the axis at height z(ψ) = r sinψ, with q·n = ±z. Segment
	// moments in the (bisector, in-ring tangent) frame are the cylinder helper's closed
	// forms; xy/y vanish by mirror symmetry about the bisector plane.
	let alpha = 0.5 * d_theta;
	let (sin_a, cos_a) = alpha.sin_cos();
	let theta_mid = 0.5 * (theta0 + theta1);
	let bis = DVec3::new(theta_mid.cos(), theta_mid.sin(), 0.0); // local coords
	let tang = DVec3::new(-theta_mid.sin(), theta_mid.cos(), 0.0);
	let outer = |u: DVec3, v: DVec3| DMat3::from_cols(u * v.x, u * v.y, u * v.z);
	let seg = |psi: f64| -> (f64, DVec3, DMat3) {
		let rho = r_maj + r_min * psi.cos();
		let z = r_min * psi.sin();
		let sigma = 0.5 * rho * rho * (d_theta - d_theta.sin());
		let q1 = 2.0 / 3.0 * rho.powi(3) * sin_a.powi(3);
		let sxx = rho.powi(4) * (0.25 * (alpha + sin_a * cos_a) - 0.5 * sin_a * cos_a.powi(3));
		let syy = rho.powi(4) * (0.25 * (alpha - sin_a * cos_a) - sin_a.powi(3) * cos_a / 6.0);
		let m1 = bis * q1 + DVec3::Z * (z * sigma);
		let t = outer(bis, bis) * sxx
			+ outer(tang, tang) * syy
			+ outer(DVec3::Z, DVec3::Z) * (z * z * sigma)
			+ (outer(bis, DVec3::Z) + outer(DVec3::Z, bis)) * (z * q1);
		// flux weights: V gets z·σ/3, m gets z·m1/4, S gets z·T/5 (± folded by caller)
		(z * sigma, m1 * z, t * z)
	};
	let (v_hi, m_hi, t_hi) = seg(psi1);
	let (v_lo, m_lo, t_lo) = seg(psi0);
	let vs = (v_hi - v_lo) / 3.0;
	let ms = (m_hi - m_lo) / 4.0;
	let ss = (t_hi - t_lo) * (1.0 / 5.0);

	// Chord-facet fan about `center` (world orientation): tetra closed forms.
	let rel: Vec<DVec3> = poly.iter().map(|&p| p - center).collect();
	let (mut fan_v, mut fan_m, mut fan_s) = (0.0_f64, DVec3::ZERO, DMat3::ZERO);
	for i in 1..rel.len() - 1 {
		let (a, b, c) = (rel[0], rel[i], rel[i + 1]);
		let vol = a.dot(b.cross(c)) / 6.0;
		fan_v += vol;
		let sm = a + b + c;
		fan_m += sm / 4.0 * vol;
		fan_s += (outer(a, a) + outer(b, b) + outer(c, c) + outer(sm, sm)) * (vol / 20.0);
	}

	// Local → world orientation (still centre-relative): v ↦ e1·vx + e2·vy + axis·vz.
	let to_world_v = |v: DVec3| e1 * v.x + e2 * v.y + axis * v.z;
	let basis = [e1, e2, axis];
	let to_world_m = |m: DMat3| {
		let cols = [m.x_axis, m.y_axis, m.z_axis];
		let mut out = DMat3::ZERO;
		for (j, col) in cols.iter().enumerate() {
			for i in 0..3 {
				out += outer(basis[i], basis[j]) * col[i];
			}
		}
		out
	};

	// Orient to the face's actual winding (canonical outward = e_r cosψ + axis sinψ),
	// exactly as torus_bulge does; the fan terms are already winding-signed.
	let centroid = poly.iter().copied().sum::<DVec3>() / poly.len() as f64;
	let relc = centroid - center;
	let hc = relc.dot(axis);
	let radial = relc - axis * hc;
	let psi_c = hc.atan2(radial.length() - r_maj);
	let outward = radial.normalize_or_zero() * psi_c.cos() + axis * psi_c.sin();
	let s = if newell_normal(poly).dot(outward) >= 0.0 { 1.0 } else { -1.0 };

	let v_l = s * (vp + vs) - fan_v;
	let m_l = to_world_v(mp + ms) * s - fan_m;
	let s_l = to_world_m(
		DMat3::from_cols(DVec3::new(sp_xx, sp_xy, sp_xz), DVec3::new(sp_xy, sp_yy, sp_yz), DVec3::new(sp_xz, sp_yz, sp_zz)) + ss,
	) * s - fan_s;

	// Shift the centre-relative lens moments to the world origin.
	let first = center * v_l + m_l;
	let second = outer(center, center) * v_l + outer(center, m_l) + outer(m_l, center) + s_l;
	(first, second)
}

/// Whether the solid's faces pass through one another — the **geometric** half of
/// validity that the half-edge invariants in [`validate`] cannot see (a closed,
/// manifold, correct-genus solid can still be geometrically invalid if two faces
/// intersect). Checked on the default tessellation; non-adjacent triangles are
/// tested for a proper crossing. A well-formed solid returns `false`.
pub fn self_intersects(s: &Solid) -> bool {
	tessellate_default(s).has_self_intersection()
}

/// Rigid-body [`MassProperties`] (volume, center of mass, inertia tensor about
/// the center of mass) at unit density. The `volume` field uses the **exact**
/// analytic [`exact_volume`] (machine-exact for planar and cylinder/sphere/cone-faced
/// solids, including holes); the center of mass is analytic for cylinder/cone/sphere/
/// torus faces (first-moment lens corrections); the inertia tensor is analytic for
/// cylindrical, spherical, conical AND toroidal faces ([`cylinder_second_moment`],
/// [`sphere_second_moment`], [`cone_second_moment`], [`torus_lens_moments`]) — a
/// cylinder, a drilled plate, a sphere, a hemisphere, a cone, a countersink, a full
/// torus, a rim-fillet band each get a machine-exact tensor. The torus correction
/// shares [`torus_bulge`]'s honesty scope: exact for θ-closed bands (whole tori,
/// fillet bands — the in-θ meridian laterals cancel pairwise); a lone toroidal face
/// spanning a partial ring keeps a documented in-θ lateral residual.
pub fn mass_properties(s: &Solid) -> MassProperties {
	let mut mp = tessellate_default(s).mass_properties();
	let exact_v = exact_volume(s);
	// Analytic centre-of-mass: the faceted mesh's first moment `∫r dV` plus the curved
	// correction for every analytic curved face (the lens between chord and true surface),
	// so a part with cylinder/sphere/cone/torus curvature — a bore, a boss, a drilled
	// plate, a filleted rim — gets an EXACT centre of mass, and mixed parts are no worse
	// than the tessellation. The faceted moment comes from the hole-aware tessellation, so
	// multi-loop (holed) PLANAR faces are handled exactly; like the bulge terms in
	// [`exact_volume`], the curved corrections span only each face's outer loop — fine
	// today because no constructor or boolean emits a curved face with inner loops. Every
	// analytic curved face additionally contributes its lens SECOND moment, making the
	// inertia tensor analytic for those parts (torus with the θ-closed-band scope
	// documented on [`torus_lens_moments`]).
	let faceted_moment = mp.center_of_mass * mp.volume;
	let mut correction = DVec3::ZERO;
	let mut covar_correction = DMat3::ZERO;
	for f in s.faces() {
		let poly = s.loop_polygon(s.face(f).outer);
		match s.face(f).surface {
			Surface::Cylinder { origin, axis, radius } => {
				let axis = axis.normalize_or_zero();
				correction += cylinder_first_moment(&poly, origin, axis, radius);
				covar_correction += cylinder_second_moment(&poly, origin, axis, radius);
			}
			Surface::Cone { apex, axis, half_angle } => {
				let axis = axis.normalize_or_zero();
				correction += cone_first_moment(&poly, apex, axis, half_angle);
				covar_correction += cone_second_moment(&poly, apex, axis, half_angle);
			}
			Surface::Sphere { center, radius } => {
				correction += sphere_first_moment(&poly, center, radius);
				covar_correction += sphere_second_moment(&poly, center, radius);
			}
			Surface::Torus { center, axis, major, minor } => {
				let (m1, m2) = torus_lens_moments(&poly, center, axis.normalize_or_zero(), major, minor);
				correction += m1;
				covar_correction += m2;
			}
			Surface::Plane { .. } => {}
		}
	}
	if exact_v.abs() > 1e-12 {
		let com = (faceted_moment + correction) / exact_v;
		// Recompose the inertia about the corrected CoM from origin-frame second moments:
		// undo the tessellation's parallel-axis shift (with ITS volume and CoM, exactly as
		// it applied it), add the lens covariance delta as `tr(ΔC)·Id − ΔC`, and shift back
		// with the exact volume and CoM. Planar-only solids round-trip unchanged (ΔC = 0,
		// exact_v = faceted volume, com = faceted CoM).
		let shift = |c: DVec3| DMat3::from_diagonal(DVec3::splat(c.length_squared())) - DMat3::from_cols(c * c.x, c * c.y, c * c.z);
		let inertia_origin = mp.inertia + shift(mp.center_of_mass) * mp.volume;
		let tr = covar_correction.x_axis.x + covar_correction.y_axis.y + covar_correction.z_axis.z;
		let delta_inertia = DMat3::from_diagonal(DVec3::splat(tr)) - covar_correction;
		mp.inertia = inertia_origin + delta_inertia - shift(com) * exact_v;
		mp.center_of_mass = com;
	}
	mp.volume = exact_v;
	mp
}

/// Structural [`SectionProperties`] of the planar cross-section cut from this solid by the
/// plane through `point` with the given `normal`: net area (bores subtracted), perimeter,
/// centroid, and the second moments of area about the centroid — what set a beam's bending
/// stiffness (`E·I`) and section modulus (`I / c`). The solid is tessellated, so the section
/// is exact for planar (prismatic) walls and converges with tessellation for curved ones.
/// `None` if the plane misses the solid or the section is degenerate.
pub fn section_properties(s: &Solid, point: DVec3, normal: DVec3) -> Option<SectionProperties> {
	tessellate_default(s).section_properties(point.as_vec3(), normal.as_vec3())
}

/// One curve of a planar cross-section of a solid: the **exact analytic conic** where the
/// crossed face's tagged surface has a closed-form plane section, or a faceted **polyline**
/// where it does not — so a section query never silently drops geometry.
#[derive(Clone, Debug)]
pub enum SectionCurve {
	/// A closed-form section conic from [`Surface::plane_section`] — exact, no meshing
	/// (line / circle / ellipse / parabola / hyperbola).
	Exact(Curve),
	/// A polyline chained from the face's chord facets, used where no closed form exists
	/// (today: oblique torus cuts — a quartic). Correct topology at chord accuracy; a
	/// closed section ring does **not** repeat its first point at the end.
	Polyline(Vec<DVec3>),
}

/// Cross-section of `s` by the plane through `plane_point` with `plane_normal`, **exact
/// where possible**: every face whose tagged surface has a closed-form plane section
/// contributes its analytic [`Curve`] (via [`Solid::section_curves`] — circles/ellipses
/// for cylinders and spheres, the full conic family for cones, concentric circles for
/// perpendicular torus cuts), and every crossed face whose surface has **no** closed form
/// (an oblique torus cut) falls back to a chained polyline over its chord facets instead
/// of being silently dropped. An AI can therefore always read a complete cross-section,
/// with exactness wherever the analytic machinery reaches.
pub fn section_curves_with_fallback(s: &Solid, plane_point: DVec3, plane_normal: DVec3) -> Vec<SectionCurve> {
	let n = plane_normal.normalize_or_zero();
	if n.length_squared() < 0.5 {
		return Vec::new();
	}
	let mut out: Vec<SectionCurve> = s.section_curves(plane_point, n).into_iter().map(SectionCurve::Exact).collect();
	// Fallback: faces that STRADDLE the plane but whose surface yields no closed-form
	// section — cut their fan facets (the tessellation convention) into segments.
	let mut segments: Vec<(DVec3, DVec3)> = Vec::new();
	for f in s.faces() {
		let poly = s.face_polygon(f);
		let sides: Vec<f64> = poly.iter().map(|p| (*p - plane_point).dot(n)).collect();
		if !(sides.iter().any(|&d| d > 1e-9) && sides.iter().any(|&d| d < -1e-9)) {
			continue; // the face does not cross the plane
		}
		if !s.face(f).surface.plane_section(plane_point, n).is_empty() {
			continue; // covered by the exact path above
		}
		for i in 1..poly.len() - 1 {
			let tri = [(poly[0], sides[0]), (poly[i], sides[i]), (poly[i + 1], sides[i + 1])];
			let mut hits: Vec<DVec3> = Vec::new();
			for k in 0..3 {
				let (a, da) = tri[k];
				let (b, db) = tri[(k + 1) % 3];
				if (da > 0.0) != (db > 0.0) && (da - db).abs() > 1e-15 {
					hits.push(a + (b - a) * (da / (da - db)));
				}
			}
			if hits.len() == 2 && (hits[0] - hits[1]).length_squared() > 1e-18 {
				segments.push((hits[0], hits[1]));
			}
		}
	}
	// Chain segments into polylines by endpoint proximity (shared facet edges cut to the
	// same point on both sides, so the weld is robust at 1e-7).
	const WELD: f64 = 1e-7;
	while let Some((a, b)) = segments.pop() {
		let mut chain = vec![a, b];
		for forward in [true, false] {
			loop {
				let end = if forward { *chain.last().unwrap() } else { chain[0] };
				let Some(idx) = segments.iter().position(|&(p, q)| (p - end).length() < WELD || (q - end).length() < WELD) else {
					break;
				};
				let (p, q) = segments.swap_remove(idx);
				let next = if (p - end).length() < WELD { q } else { p };
				if forward {
					chain.push(next);
				} else {
					chain.insert(0, next);
				}
			}
		}
		// A closed ring comes back to its start; drop the duplicate closing point.
		if chain.len() > 2 && (chain[0] - *chain.last().unwrap()).length() < WELD {
			chain.pop();
		}
		out.push(SectionCurve::Polyline(chain));
	}
	out
}

/// Moldability (draft) analysis of this solid against the mold `pull_dir` (the direction the
/// mold opens): each face's draft angle (`0°` = a wall parallel to pull, which would drag;
/// `90°` = a face square to pull), the total area below `min_draft_deg`, and the undercut
/// faces trapped between the two mold halves. A core design-for-manufacture check — an AI can
/// tell whether a part will release from a mold. The solid is tessellated, so this is exact
/// for planar walls and converges with tessellation for curved ones.
pub fn draft_analysis(s: &Solid, pull_dir: DVec3, min_draft_deg: f64) -> DraftReport {
	tessellate_default(s).draft_analysis(pull_dir.as_vec3(), min_draft_deg as f32)
}

/// Ray-based wall-thickness analysis of this solid: from each tessellated face, cast a ray
/// inward to the opposite wall and record the local material thickness; faces thinner than
/// `flag_below` are summed into the report's thin area. A core printability / castability
/// check — an AI can find the thinnest wall of a part before it is made. The solid is
/// tessellated; a through-hole or open region records [`f64::INFINITY`] for that ray.
pub fn wall_thickness(s: &Solid, flag_below: f64) -> ThicknessReport {
	tessellate_default(s).wall_thickness(flag_below)
}

/// [`wall_thickness`] with the sampler's full controls — the acute-wedge
/// (knife-edge) exclusion that keeps a dovetail lip or a cone rim out of
/// `thin_area` (see [`kernel_core::mesh::thickness`]).
pub fn wall_thickness_with(s: &Solid, opts: ThicknessOptions) -> ThicknessReport {
	tessellate_default(s).wall_thickness_with(opts)
}

/// Additive-manufacturing **overhang** analysis of this solid against the upward `build_dir`:
/// which downward-facing surface would need support material. `support_overhang_deg` is the
/// steepest overhang from vertical that still prints unsupported (a vertical wall is 0°, a
/// horizontal ceiling 90°; the common default is 45°). Reports the supported area, total area,
/// their ratio, and a per-face flag — a core 3D-printing design check. The solid is tessellated.
pub fn overhang_analysis(s: &Solid, build_dir: DVec3, support_overhang_deg: f64) -> OverhangReport {
	tessellate_default(s).overhang_analysis(build_dir.as_vec3(), support_overhang_deg as f32)
}
