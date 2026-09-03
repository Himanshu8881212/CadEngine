// Copyright (c) LMCAD. Licensed under the MIT License.

//! Triangle [`Mesh`] — the single output type of every meshing path — plus
//! correctness oracles (signed volume, surface area, manifold check) and
//! exporters (STL, OBJ, 3MF).


mod formats;
mod measure;
pub mod thickness;
pub(crate) use measure::triangle_triangle_distance;
pub use measure::SelfIntersection;
use std::collections::HashMap;
pub use thickness::{ThicknessOptions, ThicknessSample, THICKNESS_SAMPLE_BUDGET};

use crate::math::{Aabb, DMat3, DVec3, Obb, Ray, Vec2, Vec3};

/// An indexed triangle mesh.
///
/// `indices` is a flat list of triangle corner indices (length is a multiple
/// of three). `normals`, when present, is one unit normal per vertex.
#[derive(Clone, Debug, Default)]
pub struct Mesh {
	pub positions: Vec<Vec3>,
	pub indices: Vec<u32>,
	pub normals: Vec<Vec3>,
}

/// Rigid-body mass properties of a closed mesh, computed at **unit density** —
/// so [`mass`](MassProperties::volume) equals the volume; multiply
/// [`inertia`](MassProperties::inertia) by the material density for physical
/// values. Exact for a closed planar-faced solid; converges with tessellation
/// for curved ones.
#[derive(Clone, Copy, Debug)]
pub struct MassProperties {
	/// Enclosed volume — equal to the mass at unit density.
	pub volume: f64,
	/// Center of mass in model space.
	pub center_of_mass: DVec3,
	/// Inertia tensor about the center of mass, at unit density (symmetric, in
	/// model axes). Multiply by the material density for physical units.
	pub inertia: DMat3,
}

/// The principal frame of an inertia tensor: the body axes in which it is
/// diagonal, with their corresponding principal moments.
#[derive(Clone, Copy, Debug)]
pub struct PrincipalAxes {
	/// Principal moments of inertia, **ascending** (so `.x` is the smallest —
	/// the easiest axis to spin about).
	pub moments: DVec3,
	/// The three unit principal axes as the **columns** of a right-handed
	/// rotation, ordered to match `moments`.
	pub axes: DMat3,
}

/// The nearest forward intersection of a ray with a mesh surface.
#[derive(Clone, Copy, Debug)]
pub struct RayHit {
	/// Ray parameter of the hit, in units of the ray direction's length (a true
	/// distance when the direction is unit-length).
	pub t: f32,
	/// The intersection point in world space.
	pub point: Vec3,
	/// Unit geometric normal of the hit triangle (from its winding).
	pub normal: Vec3,
	/// Index of the hit triangle (its first index is `3 * triangle`).
	pub triangle: usize,
}

/// Additive-manufacturing overhang classification of a mesh against a build
/// direction — which downward-facing surface would need support material.
#[derive(Clone, Debug)]
pub struct OverhangReport {
	/// Total area of faces that overhang beyond the self-supporting threshold.
	pub overhang_area: f64,
	/// Total surface area (for context and the fraction).
	pub total_area: f64,
	/// `overhang_area / total_area` (0 for an empty mesh).
	pub overhang_fraction: f64,
	/// One flag per triangle, in index order: `true` ⟺ it needs support.
	pub needs_support: Vec<bool>,
}

/// FDM **support-necessity** classification of a mesh in a fixed print
/// orientation — the practical refinement of [`OverhangReport`], which flags
/// *every* downward face. A printer needs support only for downward surface that
/// is neither resting on the build plate nor a flat ceiling it can bridge, so
/// this report splits the flagged area into three honest buckets.
#[derive(Clone, Debug)]
pub struct SupportFreeReport {
	/// Downward-facing area whose triangles lie entirely within `bed_tol` of the
	/// lowest point — the first layer on the build plate. Never needs support.
	pub bed_area: f64,
	/// Near-horizontal ceiling area (within 1° of dead flat) above the bed —
	/// printable as bridges. Whether a bridge is *comfortable* depends on its
	/// span; see [`max_bridge_span`](Self::max_bridge_span).
	pub bridge_area: f64,
	/// Downward-facing area beyond the overhang threshold that is neither bed
	/// contact nor a flat bridge — **this is what would need support material**.
	/// A part prints support-free ⟺ this is (numerically) zero.
	pub steep_area: f64,
	/// Total surface area, for context.
	pub total_area: f64,
	/// Largest TRUE span of any connected bridge patch: 2 × the deepest interior
	/// point's distance to the patch boundary — a Ø10 disc ceiling spans 10, a
	/// 300×8 slot spans 8, and an annular ring spans its RADIAL WIDTH (both the
	/// AABB-diagonal and min-projected-extent metrics before it over-reported
	/// annuli at their full diameter — no projection can see a hole).
	/// FDM handles ~5–10 mm trivially; long spans droop. 0 when none.
	pub max_bridge_span: f64,
	/// One flag per triangle, in index order: `true` ⟺ it needs support (the
	/// per-triangle version of [`steep_area`](Self::steep_area)).
	pub steep: Vec<bool>,
	/// WHERE the steep area is: centroids of the largest flagged triangles
	/// (up to 8, largest-area first). A failing support budget names its
	/// offending feature instead of leaving the author to reason blind.
	pub steep_exemplars: Vec<Vec3>,
	/// Per connected bridge patch: `(span, interior exemplar point)`, sorted
	/// widest first (up to 8). `max_bridge_span` is `bridge_patches[0].0`.
	pub bridge_patches: Vec<(f64, Vec3)>,
}

/// Structural properties of a planar cross-section: its (net, holes-subtracted)
/// area and perimeter, centroid, and the second moments of area about the
/// centroid in the section plane's `(u, v)` basis — what sets a beam's bending
/// stiffness and section modulus.
#[derive(Clone, Copy, Debug)]
pub struct SectionProperties {
	/// Net enclosed area (holes subtracted).
	pub area: f64,
	/// Total boundary length over all contours (outer and holes).
	pub perimeter: f64,
	/// Centroid (area-weighted) as a 3-D point on the section plane.
	pub centroid: Vec3,
	/// `∫(u − c_u)² dA` about the centroid.
	pub i_uu: f64,
	/// `∫(v − c_v)² dA` about the centroid.
	pub i_vv: f64,
	/// `∫(u − c_u)(v − c_v) dA` about the centroid (product of area).
	pub i_uv: f64,
	/// Farthest boundary fibre from the centroid — `σ = M·c_max/I` gives the
	/// classic worst bending stress of the section.
	pub c_max: f64,
	/// The in-plane basis the moments are expressed in (`u_axis × v_axis = normal`).
	pub u_axis: Vec3,
	/// See [`u_axis`](Self::u_axis).
	pub v_axis: Vec3,
}

/// Moldability analysis against a mold pull (draw) direction: per-face draft
/// angle, faces with insufficient draft, and undercuts (faces trapped between
/// the two mold halves). The two defects that stop a part being injection-molded
/// or cast.
#[derive(Clone, Debug)]
pub struct DraftReport {
	/// Smallest draft angle (degrees) over all faces.
	pub min_draft_deg: f64,
	/// Per-triangle draft angle (degrees from the pull-perpendicular plane), in
	/// index order — 0° is a wall parallel to pull, 90° is a face square to it.
	pub draft_deg: Vec<f64>,
	/// Total area of faces whose draft is below the requested minimum.
	pub low_draft_area: f64,
	/// Total area of undercut faces (occluded along both pull directions).
	pub undercut_area: f64,
	/// Per-triangle undercut flag, in index order.
	pub undercut: Vec<bool>,
}

/// Ray-based wall-thickness analysis of a mesh — how thick the material is under
/// each face, and where it is thinner than a printable/moldable minimum. Produced
/// by [`Mesh::wall_thickness`] / [`Mesh::wall_thickness_with`]; the sampling
/// contract is documented on [`mesh::thickness`](crate::mesh::thickness).
#[derive(Clone, Debug)]
pub struct ThicknessReport {
	/// Smallest wall thickness over the COUNTED samples (every sample when no
	/// wedge exclusion is set; the non-wedge samples otherwise).
	pub min_thickness: f64,
	/// Per-triangle thickness (inward ray from the triangle's centroid to the
	/// opposite wall), in index order. [`f64::INFINITY`] where the inward ray
	/// found no opposite wall. Unaffected by the wedge exclusion.
	pub thickness: Vec<f64>,
	/// Total area (area-weighted stratified samples) thinner than the queried
	/// minimum, EXCLUDING the acute-wedge readings when an exclusion is set.
	pub thin_area: f64,
	/// Area thinner than the queried minimum whose reading is an acute-wedge
	/// (knife-edge) reading — the ray left through a face that meets the sample's
	/// own face at a convex material angle below `exclude_wedge_deg`. Always `0`
	/// when no exclusion is set (those samples then count in `thin_area`).
	pub thin_area_wedge: f64,
	/// Every surface sample taken, in deterministic order (triangle order, then
	/// the triangle's stratified sub-cells).
	pub samples: Vec<ThicknessSample>,
	/// The wedge exclusion the report was computed with, if any.
	pub exclude_wedge_deg: Option<f64>,
}

/// The nearest point on a mesh surface to a query point.
#[derive(Clone, Copy, Debug)]
pub struct ClosestPoint {
	/// The closest surface point.
	pub point: Vec3,
	/// Euclidean distance from the query to `point`.
	pub distance: f32,
	/// Index of the triangle carrying the closest point.
	pub triangle: usize,
}

impl MassProperties {
	/// Diagonalize the [`inertia`](Self::inertia) tensor into its
	/// [`PrincipalAxes`] (principal moments ascending + the body frame that
	/// diagonalizes it). Uses cyclic Jacobi rotations, which converge for any
	/// real symmetric matrix; the axes are made right-handed.
	pub fn principal_axes(&self) -> PrincipalAxes {
		let m = self.inertia;
		let c = [m.x_axis, m.y_axis, m.z_axis]; // columns; a[i][j] = c[j][i]
		let a = [
			[c[0].x, c[1].x, c[2].x],
			[c[0].y, c[1].y, c[2].y],
			[c[0].z, c[1].z, c[2].z],
		];
		let (vals, vecs) = jacobi_eigen_symmetric(a);
		// Sort the three eigenpairs by ascending moment.
		let mut order = [0usize, 1, 2];
		order.sort_by(|&i, &j| vals[i].total_cmp(&vals[j]));
		let col = |p: usize| DVec3::new(vecs[0][p], vecs[1][p], vecs[2][p]).normalize_or_zero();
		let (e0, e1) = (col(order[0]), col(order[1]));
		// Force a right-handed frame: the third axis follows from the first two.
		let e2 = e0.cross(e1).normalize_or_zero();
		PrincipalAxes {
			moments: DVec3::new(vals[order[0]], vals[order[1]], vals[order[2]]),
			axes: DMat3::from_cols(e0, e1, e2),
		}
	}

	/// Exact closed-form properties of a **solid axis-aligned box** of full extents
	/// `dx`×`dy`×`dz`, centered at the origin, at unit density (mass = volume). The
	/// inertia about the center is diagonal: `Iₓₓ = m(dy²+dz²)/12`, cyclically. Unlike the
	/// tessellation path this is machine-exact and needs no mesh.
	pub fn solid_box(dx: f64, dy: f64, dz: f64) -> MassProperties {
		let m = dx * dy * dz;
		MassProperties {
			volume: m,
			center_of_mass: DVec3::ZERO,
			inertia: DMat3::from_diagonal(
				DVec3::new(dy * dy + dz * dz, dx * dx + dz * dz, dx * dx + dy * dy) * (m / 12.0),
			),
		}
	}

	/// Exact closed-form properties of a **solid cylinder** of `radius` and `height` whose
	/// axis is **+Z**, centered at the origin, at unit density. About the center:
	/// `I_zz = ½ m r²` (the spin axis) and `I_xx = I_yy = m(3r² + h²)/12`. Exact where the
	/// faceted tessellation only converges.
	pub fn solid_cylinder(radius: f64, height: f64) -> MassProperties {
		let m = core::f64::consts::PI * radius * radius * height;
		let axial = 0.5 * m * radius * radius;
		let radial = m * (3.0 * radius * radius + height * height) / 12.0;
		MassProperties {
			volume: m,
			center_of_mass: DVec3::ZERO,
			inertia: DMat3::from_diagonal(DVec3::new(radial, radial, axial)),
		}
	}

	/// Exact closed-form properties of a **solid sphere** of `radius`, centered at the
	/// origin, at unit density. Isotropic: `I = ⅖ m r²` about every axis through the center.
	pub fn solid_sphere(radius: f64) -> MassProperties {
		let m = 4.0 / 3.0 * core::f64::consts::PI * radius * radius * radius;
		MassProperties {
			volume: m,
			center_of_mass: DVec3::ZERO,
			inertia: DMat3::from_diagonal(DVec3::splat(0.4 * m * radius * radius)),
		}
	}

	/// Translate the whole rigid body by `offset`: the center of mass shifts by `offset`
	/// while the inertia — reported **about the center of mass** — is translation-invariant
	/// and unchanged. Use to place a part's properties at its position before [`combine`].
	///
	/// [`combine`]: Self::combine
	pub fn translated(self, offset: DVec3) -> MassProperties {
		MassProperties { center_of_mass: self.center_of_mass + offset, ..self }
	}

	/// Place this body under a **rigid** pose: rotate by `rotation` (assumed orthonormal —
	/// no scale) then translate by `translation`. The center of mass maps as a point
	/// (`R·c + t`) and the inertia tensor rotates as `R·I·Rᵀ`; the volume is unchanged. Use
	/// to bring a part's local [`MassProperties`] into an assembly's world frame before
	/// [`combine`](Self::combine).
	pub fn transformed(self, rotation: DMat3, translation: DVec3) -> MassProperties {
		MassProperties {
			volume: self.volume,
			center_of_mass: rotation * self.center_of_mass + translation,
			inertia: rotation * self.inertia * rotation.transpose(),
		}
	}

	/// Exact rigid-body composition of `parts` into a single body at unit density, via the
	/// parallel-axis theorem: volumes add, the combined center of mass is the volume-weighted
	/// mean, and each part's inertia is shifted from its own center to the combined center
	/// before summing. Lets an assembly's total mass, balance point and inertia be computed
	/// exactly from its parts' [`MassProperties`] without re-meshing the whole. Assumes the
	/// parts do not overlap (otherwise the shared material is double-counted).
	pub fn combine(parts: &[MassProperties]) -> MassProperties {
		let volume: f64 = parts.iter().map(|p| p.volume).sum();
		if volume.abs() < 1e-12 {
			return MassProperties { volume: 0.0, center_of_mass: DVec3::ZERO, inertia: DMat3::ZERO };
		}
		let com = parts.iter().fold(DVec3::ZERO, |acc, p| acc + p.center_of_mass * p.volume) / volume;
		let mut inertia = DMat3::ZERO;
		for p in parts {
			// Parallel-axis shift of part `p`'s inertia from its own CoM to the combined CoM.
			let d = p.center_of_mass - com;
			let outer = DMat3::from_cols(d * d.x, d * d.y, d * d.z);
			let shift = DMat3::from_diagonal(DVec3::splat(d.length_squared())) - outer;
			inertia += p.inertia + shift * p.volume;
		}
		MassProperties { volume, center_of_mass: com, inertia }
	}
}

/// Eigenvalues and eigenvectors (as columns) of a real symmetric 3×3 matrix via
/// cyclic Jacobi rotations. `a` is assumed symmetric; only the rotation count is
/// bounded (the 3×3 case converges in a handful of sweeps).
fn jacobi_eigen_symmetric(mut a: [[f64; 3]; 3]) -> ([f64; 3], [[f64; 3]; 3]) {
	let mut v = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
	for _ in 0..64 {
		let off = a[0][1].abs() + a[0][2].abs() + a[1][2].abs();
		let scale = (a[0][0].abs() + a[1][1].abs() + a[2][2].abs()).max(1.0);
		if off <= f64::EPSILON * scale {
			break; // already diagonal to machine precision
		}
		for (p, q) in [(0usize, 1usize), (0, 2), (1, 2)] {
			let apq = a[p][q];
			if apq == 0.0 {
				continue;
			}
			// Rotation that zeros a[p][q] (Numerical Recipes formulation).
			let theta = (a[q][q] - a[p][p]) / (2.0 * apq);
			let t = if theta == 0.0 {
				1.0
			} else {
				theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt())
			};
			let cos = 1.0 / (t * t + 1.0).sqrt();
			let sin = t * cos;
			let r = 3 - p - q; // the index that is neither p nor q
			let (arp, arq) = (a[r][p], a[r][q]);
			a[p][p] -= t * apq;
			a[q][q] += t * apq;
			a[p][q] = 0.0;
			a[q][p] = 0.0;
			a[r][p] = cos * arp - sin * arq;
			a[p][r] = a[r][p];
			a[r][q] = sin * arp + cos * arq;
			a[q][r] = a[r][q];
			for vk in v.iter_mut() {
				let (vkp, vkq) = (vk[p], vk[q]);
				vk[p] = cos * vkp - sin * vkq;
				vk[q] = sin * vkp + cos * vkq;
			}
		}
	}
	([a[0][0], a[1][1], a[2][2]], v)
}

/// A finite coordinate, or `0.0` for a non-finite one — so exporters never emit a
/// `NaN`/`inf` token into a text/JSON format that cannot represent it.
fn finite_or_zero(v: f32) -> f32 {
	if v.is_finite() {
		v
	} else {
		0.0
	}
}

/// The closest point to `p` on triangle `a,b,c`, by Voronoi-region test
/// (Ericson, *Real-Time Collision Detection*). Returns a vertex, an edge point,
/// or an interior point as appropriate — the building block of [`Mesh::closest_point`].
pub fn closest_point_on_triangle(p: Vec3, a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
	let ab = b - a;
	let ac = c - a;
	let ap = p - a;
	let d1 = ab.dot(ap);
	let d2 = ac.dot(ap);
	if d1 <= 0.0 && d2 <= 0.0 {
		return a;
	}
	let bp = p - b;
	let d3 = ab.dot(bp);
	let d4 = ac.dot(bp);
	if d3 >= 0.0 && d4 <= d3 {
		return b;
	}
	let vc = d1 * d4 - d3 * d2;
	if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
		let v = d1 / (d1 - d3);
		return a + ab * v;
	}
	let cp = p - c;
	let d5 = ab.dot(cp);
	let d6 = ac.dot(cp);
	if d6 >= 0.0 && d5 <= d6 {
		return c;
	}
	let vb = d5 * d2 - d1 * d6;
	if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
		let w = d2 / (d2 - d6);
		return a + ac * w;
	}
	let va = d3 * d6 - d5 * d4;
	if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
		let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		return b + (c - b) * w;
	}
	let denom = 1.0 / (va + vb + vc);
	let v = vb * denom;
	let w = vc * denom;
	a + ab * v + ac * w
}

/// Area and area-moment integrals of a simple polygon (CCW ⇒ positive area),
/// by Green's theorem: returns `(A, ∫u dA, ∫v dA, ∫u² dA, ∫v² dA, ∫uv dA)`.
fn polygon_moments(poly: &[Vec2]) -> (f64, f64, f64, f64, f64, f64) {
	let (mut a, mut mx, mut my, mut sxx, mut syy, mut sxy) = (0.0f64, 0.0, 0.0, 0.0, 0.0, 0.0);
	let n = poly.len();
	for k in 0..n {
		let p = poly[k];
		let q = poly[(k + 1) % n];
		let (px, py, qx, qy) = (p.x as f64, p.y as f64, q.x as f64, q.y as f64);
		let cross = px * qy - qx * py;
		a += cross;
		mx += (px + qx) * cross;
		my += (py + qy) * cross;
		sxx += (px * px + px * qx + qx * qx) * cross;
		syy += (py * py + py * qy + qy * qy) * cross;
		sxy += (px * qy + 2.0 * px * py + 2.0 * qx * qy + qx * py) * cross;
	}
	(a / 2.0, mx / 6.0, my / 6.0, sxx / 12.0, syy / 12.0, sxy / 24.0)
}

/// Even–odd point-in-polygon test (2-D ray crossing).
fn point_in_poly_2d(p: Vec2, poly: &[Vec2]) -> bool {
	let mut inside = false;
	let n = poly.len();
	let mut j = n - 1;
	for i in 0..n {
		let (a, b) = (poly[i], poly[j]);
		if (a.y > p.y) != (b.y > p.y) {
			let x = a.x + (p.y - a.y) / (b.y - a.y) * (b.x - a.x);
			if p.x < x {
				inside = !inside;
			}
		}
		j = i;
	}
	inside
}

/// Nearest forward ray–triangle intersection (Möller–Trumbore), as
/// `(t, point, unit_normal)`, or `None` if the ray misses or hits behind the
/// origin. `t` is in units of the ray direction's length. Shared by
/// [`Mesh::raycast`] and the BVH.
pub(crate) fn ray_triangle(ray: Ray, a: Vec3, b: Vec3, c: Vec3) -> Option<(f32, Vec3, Vec3)> {
	let (o, d) = (ray.origin, ray.dir);
	let eps = 1e-7f32;
	let (e1, e2) = (b - a, c - a);
	let pv = d.cross(e2);
	let det = e1.dot(pv);
	if det.abs() < eps {
		return None; // ray parallel to the triangle
	}
	let inv = 1.0 / det;
	let tv = o - a;
	let u = tv.dot(pv) * inv;
	if !(0.0..=1.0).contains(&u) {
		return None;
	}
	let qv = tv.cross(e1);
	let v = d.dot(qv) * inv;
	if v < 0.0 || u + v > 1.0 {
		return None;
	}
	let t = e2.dot(qv) * inv;
	if t <= eps {
		return None; // behind the origin
	}
	Some((t, o + d * t, e1.cross(e2).normalize_or_zero()))
}


/// What a mesh-level heal ([`Mesh::fill_holes`] / [`Mesh::weld`]) actually did to
/// the geometry — the core-mesh analogue of the B-rep [`heal::HealReport`]. A bare
/// hole count or a silent `weld` hides that a repair can invent an interior (a
/// filled channel jumps the volume) or move a wall (the area/volume shift): this
/// report ANNOUNCES the change to direct callers so it is never silent.
#[derive(Clone, Debug, PartialEq)]
pub struct MeshHealDelta {
	/// Which heal ran: `"fill_holes"` or `"weld"`.
	pub op: &'static str,
	/// Triangle count before / after.
	pub triangles_before: usize,
	pub triangles_after: usize,
	/// Vertex count before / after (drops on a weld that merges duplicates).
	pub vertices_before: usize,
	pub vertices_after: usize,
	/// `fill_holes`: number of boundary loops capped.
	pub holes_filled: usize,
	/// `fill_holes`: edge count of the LARGEST boundary loop closed (the biggest
	/// single opening filled); 0 for a weld.
	pub largest_opening_edges: usize,
	/// Unpaired (boundary) half-edges before / after — the open-crack measure
	/// (0 after a successful fill).
	pub open_edges_before: usize,
	pub open_edges_after: usize,
	/// Signed volume before / after: a jump reveals an invented interior (a
	/// closed-off channel) — the honesty signal a bare count can't give.
	pub signed_volume_before: f64,
	pub signed_volume_after: f64,
	/// Surface area before / after: a shift reveals a moved wall.
	pub surface_area_before: f64,
	pub surface_area_after: f64,
}

impl MeshHealDelta {
	/// True if the heal changed the geometry at all (topology or measure).
	pub fn changed_geometry(&self) -> bool {
		self.triangles_before != self.triangles_after
			|| self.vertices_before != self.vertices_after
			|| (self.signed_volume_after - self.signed_volume_before).abs() > 1e-9
			|| (self.surface_area_after - self.surface_area_before).abs() > 1e-9
	}

	/// Change in enclosed volume — a large positive delta from `fill_holes` means
	/// a channel/void was sealed off (interior invented), not a rim tidied.
	pub fn volume_delta(&self) -> f64 {
		self.signed_volume_after - self.signed_volume_before
	}
}

/// How many triangles traverse each directed edge `(a, b)`. An undirected edge
/// `{a, b}` is *open* when the two entries sum to 1, *closed* when they sum to
/// 2, and *non-orientable* when one of them alone is 2.
type DirectedEdgeUses = HashMap<(u32, u32), u32>;

impl Mesh {
	/// Every directed edge of the mesh with how many triangles traverse it, plus
	/// the directed edges in first-appearance (triangle) order.
	///
	/// The ordered list exists so boundary walks are deterministic: a hash
	/// container's iteration order is seeded per process, and it decides which
	/// loop a walk starts on and where it splices at a pinch vertex (see
	/// [`Mesh::fill_holes`], whose repair must reproduce byte for byte).
	fn directed_edges(&self) -> (DirectedEdgeUses, Vec<(u32, u32)>) {
		let mut dir: HashMap<(u32, u32), u32> = HashMap::new();
		let mut ordered: Vec<(u32, u32)> = Vec::with_capacity(self.indices.len());
		for t in self.indices.chunks_exact(3) {
			for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
				let n = dir.entry((a, b)).or_insert(0);
				if *n == 0 {
					ordered.push((a, b));
				}
				*n += 1;
			}
		}
		(dir, ordered)
	}

	/// True iff the undirected edge `{a, b}` is used by exactly ONE triangle —
	/// i.e. it lies on an open rim.
	fn is_boundary_edge(dir: &DirectedEdgeUses, a: u32, b: u32) -> bool {
		dir.get(&(a, b)).copied().unwrap_or(0) + dir.get(&(b, a)).copied().unwrap_or(0) == 1
	}

	/// Boundary (open-crack) edge count: undirected edges used by exactly one
	/// triangle. Every such edge lies on a hole rim.
	///
	/// # Why this is not "the reverse edge is missing"
	///
	/// It used to be, and that is a different — wrong — question. Two triangles
	/// that share an edge but wind the SAME way (`a→b` twice, `b→a` never) close
	/// that edge perfectly: no rim, no crack, nothing to fill. They are
	/// *non-orientable*, which is a winding defect, not an opening. Asking "is
	/// the reverse present?" reported all of them as boundary and put this
	/// oracle in permanent contradiction with [`crate::meshcheck::check_mesh`],
	/// which has always counted boundary (used once) and non-orientable (used
	/// twice, same direction) separately. Anything gating on "the measurement
	/// surface is not closed" then fired on solids whose tessellation is closed —
	/// which is exactly how it reached 11 shipped part programs across 8 campaigns
	/// as a false "the faceter dropped geometry" refusal (2026-08-08). Use
	/// [`Mesh::is_two_manifold`] when orientability matters too.
	pub fn boundary_edge_count(&self) -> usize {
		let (dir, ordered) = self.directed_edges();
		ordered.iter().filter(|&&(a, b)| Self::is_boundary_edge(&dir, a, b)).count()
	}

	/// Non-orientable edge count: undirected edges whose two triangles traverse
	/// them the SAME way, i.e. one of the pair is wound inside-out.
	///
	/// Closure and orientability are different defects and this method exists so
	/// they can be reported apart without paying for
	/// [`crate::check_mesh`](crate::check_mesh)'s self-intersection sweep. Same
	/// edge-hash pass as [`Mesh::boundary_edge_count`], same answer as
	/// `check_mesh().non_orientable_edges`.
	pub fn non_orientable_edge_count(&self) -> usize {
		let (dir, ordered) = self.directed_edges();
		let mut seen: std::collections::HashSet<(u32, u32)> = std::collections::HashSet::new();
		let mut count = 0usize;
		for &(a, b) in &ordered {
			let key = if a < b { (a, b) } else { (b, a) };
			if !seen.insert(key) {
				continue; // one visit per undirected edge
			}
			let fwd = dir.get(&key).copied().unwrap_or(0);
			let bwd = dir.get(&(key.1, key.0)).copied().unwrap_or(0);
			if fwd + bwd == 2 && (fwd == 2 || bwd == 2) {
				count += 1;
			}
		}
		count
	}

	/// Midpoints of up to `cap` non-orientable edges (same defect as
	/// [`Mesh::non_orientable_edge_count`]), so a nonzero count is locatable
	/// instead of just countable — a receipt that says "105 non-orientable
	/// edges" should point at them. Deterministic: first-encounter order of the
	/// triangle scan.
	pub fn non_orientable_edge_witnesses(&self, cap: usize) -> Vec<[f64; 3]> {
		self.edge_witnesses(cap, |fwd, bwd| fwd + bwd == 2 && (fwd == 2 || bwd == 2))
	}

	/// Midpoints of up to `cap` boundary (open-rim) edges — the locatable form
	/// of [`Mesh::boundary_edge_count`]. Same traversal order as
	/// [`Mesh::non_orientable_edge_witnesses`].
	pub fn boundary_edge_witnesses(&self, cap: usize) -> Vec<[f64; 3]> {
		self.edge_witnesses(cap, |fwd, bwd| fwd + bwd == 1)
	}

	/// Midpoints of up to `cap` non-manifold edges (undirected edges used by
	/// more than two triangles — a fin or T-junction). Same traversal order as
	/// [`Mesh::non_orientable_edge_witnesses`].
	pub fn non_manifold_edge_witnesses(&self, cap: usize) -> Vec<[f64; 3]> {
		self.edge_witnesses(cap, |fwd, bwd| fwd + bwd > 2)
	}

	/// Midpoints of the first `cap` undirected edges whose directed use counts
	/// `(forward, backward)` satisfy `offending`, in first-encounter order of the
	/// triangle scan — the one traversal behind every edge-witness list, so the
	/// three defect kinds are located the same way.
	fn edge_witnesses(&self, cap: usize, offending: impl Fn(u32, u32) -> bool) -> Vec<[f64; 3]> {
		let (dir, ordered) = self.directed_edges();
		let mut seen: std::collections::HashSet<(u32, u32)> = std::collections::HashSet::new();
		let mut out = Vec::new();
		for &(a, b) in &ordered {
			let key = if a < b { (a, b) } else { (b, a) };
			if !seen.insert(key) {
				continue;
			}
			let fwd = dir.get(&key).copied().unwrap_or(0);
			let bwd = dir.get(&(key.1, key.0)).copied().unwrap_or(0);
			if offending(fwd, bwd) {
				let (p, q) = (self.positions[a as usize], self.positions[b as usize]);
				let m = (p.as_dvec3() + q.as_dvec3()) * 0.5;
				out.push([m.x, m.y, m.z]);
				if out.len() >= cap {
					break;
				}
			}
		}
		out
	}

	/// The edge count of the longest boundary loop (the largest single opening).
	fn largest_boundary_loop(&self) -> usize {
		let (dir, ordered) = self.directed_edges();
		let mut next: std::collections::HashMap<u32, u32> = std::collections::HashMap::new();
		let mut starts: Vec<u32> = Vec::new();
		for &(a, b) in &ordered {
			if Self::is_boundary_edge(&dir, a, b) && next.insert(a, b).is_none() {
				starts.push(a); // boundary edge a->b
			}
		}
		let mut visited: std::collections::HashSet<u32> = std::collections::HashSet::new();
		let mut best = 0usize;
		for start in starts {
			if visited.contains(&start) {
				continue;
			}
			let (mut v, mut len) = (start, 0usize);
			while let Some(&w) = next.get(&v) {
				if visited.contains(&v) {
					break;
				}
				visited.insert(v);
				len += 1;
				v = w;
				if v == start {
					break;
				}
			}
			best = best.max(len);
		}
		best
	}

	/// [`fill_holes`](Self::fill_holes) that ANNOUNCES what it changed — the
	/// geometry-delta report a bare hole count hides (a sealed channel jumps the
	/// volume). Same repair, deterministic; measures before/after around it.
	pub fn fill_holes_reported(&mut self) -> MeshHealDelta {
		let (tb, vb) = (self.triangle_count(), self.vertex_count());
		let (vol_b, area_b) = (self.signed_volume(), self.surface_area());
		let (open_b, largest) = (self.boundary_edge_count(), self.largest_boundary_loop());
		let holes = self.fill_holes();
		MeshHealDelta {
			op: "fill_holes",
			triangles_before: tb,
			triangles_after: self.triangle_count(),
			vertices_before: vb,
			vertices_after: self.vertex_count(),
			holes_filled: holes,
			largest_opening_edges: largest,
			open_edges_before: open_b,
			open_edges_after: self.boundary_edge_count(),
			signed_volume_before: vol_b,
			signed_volume_after: self.signed_volume(),
			surface_area_before: area_b,
			surface_area_after: self.surface_area(),
		}
	}

	/// [`weld`](Self::weld) that ANNOUNCES what it changed — the vertices merged
	/// and any area/volume shift from dropped needle triangles or a moved wall,
	/// instead of returning `()` silently.
	pub fn weld_reported(&mut self, tolerance: f32) -> MeshHealDelta {
		let (tb, vb) = (self.triangle_count(), self.vertex_count());
		let (vol_b, area_b) = (self.signed_volume(), self.surface_area());
		let open_b = self.boundary_edge_count();
		self.weld(tolerance);
		MeshHealDelta {
			op: "weld",
			triangles_before: tb,
			triangles_after: self.triangle_count(),
			vertices_before: vb,
			vertices_after: self.vertex_count(),
			holes_filled: 0,
			largest_opening_edges: 0,
			open_edges_before: open_b,
			open_edges_after: self.boundary_edge_count(),
			signed_volume_before: vol_b,
			signed_volume_after: self.signed_volume(),
			surface_area_before: area_b,
			surface_area_after: self.surface_area(),
		}
	}
	pub fn new() -> Self {
		Self::default()
	}

	pub fn vertex_count(&self) -> usize {
		self.positions.len()
	}

	pub fn triangle_count(&self) -> usize {
		self.indices.len() / 3
	}

	pub fn is_empty(&self) -> bool {
		self.indices.is_empty()
	}

	/// Append a vertex, returning its index.
	pub fn push_vertex(&mut self, p: Vec3) -> u32 {
		let i = self.positions.len() as u32;
		self.positions.push(p);
		i
	}

	/// Append a triangle from existing vertex indices.
	pub fn push_triangle(&mut self, a: u32, b: u32, c: u32) {
		self.indices.extend_from_slice(&[a, b, c]);
	}

	/// Iterate triangles as `[u32; 3]` index triples.
	pub fn triangles(&self) -> impl Iterator<Item = [u32; 3]> + '_ {
		self.indices.chunks_exact(3).map(|t| [t[0], t[1], t[2]])
	}

	/// Whether any two **non-adjacent** triangles properly intersect — the geometric
	/// half of solid validity (a closed, manifold solid is still invalid if its faces
	/// pass through one another). Triangles sharing a vertex index are skipped (a shared
	/// edge/vertex is legitimate adjacency, not a self-intersection), and an
	/// axis-aligned bounding-box test rejects distant pairs before the exact triangle
	/// test. Coplanar overlap is intentionally not counted (it is not a proper
	/// crossing). Candidate pairs come from a triangle BVH (see
	/// [`crate::meshcheck`]), so a clean mesh costs ~O(T log T), not O(T²).
	pub fn has_self_intersection(&self) -> bool {
		crate::meshcheck::has_proper_self_intersection(self)
	}

	/// Signed volume via the divergence theorem (sum of tetra triple products).
	///
	/// Positive for an outward (CCW) winding of a closed surface — used as the
	/// orientation oracle. Accumulated in `f64` for precision.
	pub fn signed_volume(&self) -> f64 {
		if self.indices.len() < 3 {
			return 0.0;
		}
		// Sum tetrahedra from a reference point *on the mesh* rather than the world
		// origin. The volume is translation-invariant, but anchoring near the
		// geometry keeps the `a·(b×c)` terms at model scale — far from the origin
		// (large-coordinate parts) the origin-anchored form catastrophically cancels
		// (terms ~|p|³ for a result ~feature³).
		let o = self.positions[self.indices[0] as usize].as_dvec3();
		let mut v = 0.0f64;
		for t in self.indices.chunks_exact(3) {
			let a = self.positions[t[0] as usize].as_dvec3() - o;
			let b = self.positions[t[1] as usize].as_dvec3() - o;
			let c = self.positions[t[2] as usize].as_dvec3() - o;
			v += a.dot(b.cross(c));
		}
		v / 6.0
	}

	/// Total triangle area, accumulated in `f64`.
	pub fn surface_area(&self) -> f64 {
		let mut area = 0.0f64;
		for t in self.indices.chunks_exact(3) {
			let a = self.positions[t[0] as usize].as_dvec3();
			let b = self.positions[t[1] as usize].as_dvec3();
			let c = self.positions[t[2] as usize].as_dvec3();
			area += (b - a).cross(c - a).length() * 0.5;
		}
		area
	}

	/// Rigid-body [`MassProperties`] (volume, center of mass, inertia tensor) at
	/// unit density. Reduces the solid integrals to a signed sum over the tetrahedra
	/// spanned by the origin and each triangle (the same divergence-theorem trick as
	/// [`signed_volume`](Self::signed_volume)), so it is exact for a closed
	/// planar-faced solid and converges with tessellation for curved ones. The mesh
	/// must be closed and outward-oriented; an empty or degenerate one yields zeros.
	pub fn mass_properties(&self) -> MassProperties {
		// Second moment of the canonical tetrahedron {u ≥ 0, Σu ≤ 1}:
		// ∫ uᵢuⱼ du is 1/60 on the diagonal and 1/120 off it, i.e.
		// (1/120)·[[2,1,1],[1,2,1],[1,1,2]].
		let canon = DMat3::from_cols(
			DVec3::new(2.0, 1.0, 1.0),
			DVec3::new(1.0, 2.0, 1.0),
			DVec3::new(1.0, 1.0, 2.0),
		) * (1.0 / 120.0);
		let mut vol6 = 0.0f64; // Σ det  =  6·volume
		let mut moment = DVec3::ZERO; // Σ det·(a+b+c)  =  24·∫ p dV
		let mut covar = DMat3::ZERO; // Σ det·J·canon·Jᵀ  =  ∫ p·pᵀ dV
		for t in self.indices.chunks_exact(3) {
			let a = self.positions[t[0] as usize].as_dvec3();
			let b = self.positions[t[1] as usize].as_dvec3();
			let c = self.positions[t[2] as usize].as_dvec3();
			let det = a.dot(b.cross(c)); // 6·signed volume of tet (O, a, b, c)
			vol6 += det;
			moment += (a + b + c) * det;
			let j = DMat3::from_cols(a, b, c);
			covar += (j * canon * j.transpose()) * det;
		}
		let volume = vol6 / 6.0;
		if volume.abs() < 1e-12 {
			return MassProperties { volume: 0.0, center_of_mass: DVec3::ZERO, inertia: DMat3::ZERO };
		}
		let com = moment / (24.0 * volume);
		// Inertia about the origin: I = trace(C)·Id − C.
		let trace = covar.x_axis.x + covar.y_axis.y + covar.z_axis.z;
		let inertia_o = DMat3::from_diagonal(DVec3::splat(trace)) - covar;
		// Parallel-axis shift to the center of mass (mass = volume at unit density).
		let outer = DMat3::from_cols(com * com.x, com * com.y, com * com.z);
		let shift = DMat3::from_diagonal(DVec3::splat(com.length_squared())) - outer;
		let inertia = inertia_o - shift * volume;
		MassProperties { volume, center_of_mass: com, inertia }
	}

	/// Cross-section of the mesh by the plane through `point` with the given
	/// `normal`, as a set of contour loops (each an ordered ring of points, the
	/// first not repeated at the end). For a closed solid every contour is a closed
	/// ring wound counter-clockwise as seen from `+normal`; an open mesh can yield
	/// open polylines. Triangles lying in the plane are skipped.
	///
	/// Robust to vertices that touch the plane: a half-open `≥ 0` side test classes
	/// an on-plane vertex identically for every triangle that shares it, so adjacent
	/// triangles always agree and each crossing triangle contributes exactly one
	/// segment. Useful for technical sections, slicing for 3-D printing, and
	/// measuring section area/perimeter.
	pub fn cross_section(&self, point: Vec3, normal: Vec3) -> Vec<Vec<Vec3>> {
		let n = normal.normalize_or_zero();
		if n == Vec3::ZERO || self.indices.is_empty() {
			return Vec::new();
		}
		let weld = (self.aabb().size().length().max(1.0) * 1e-5) as f64;

		// 1) One crossing segment per transversal triangle.
		let mut segs: Vec<[Vec3; 2]> = Vec::new();
		for t in self.indices.chunks_exact(3) {
			let p = [
				self.positions[t[0] as usize],
				self.positions[t[1] as usize],
				self.positions[t[2] as usize],
			];
			let d = [
				(p[0] - point).dot(n) as f64,
				(p[1] - point).dot(n) as f64,
				(p[2] - point).dot(n) as f64,
			];
			let above = [d[0] >= 0.0, d[1] >= 0.0, d[2] >= 0.0];
			if above[0] == above[1] && above[1] == above[2] {
				continue; // wholly on one side, or coplanar
			}
			let mut pts: Vec<Vec3> = Vec::with_capacity(2);
			for (i, j) in [(0usize, 1usize), (1, 2), (2, 0)] {
				if above[i] != above[j] {
					let s = (d[i] / (d[i] - d[j])) as f32;
					pts.push(p[i].lerp(p[j], s));
				}
			}
			if pts.len() == 2 && pts[0].distance(pts[1]) as f64 > weld {
				segs.push([pts[0], pts[1]]);
			}
		}
		if segs.is_empty() {
			return Vec::new();
		}

		// 2) Weld endpoints (the two triangles sharing an edge produce the same
		// crossing up to f32 rounding) into shared contour vertices.
		let mut verts: Vec<Vec3> = Vec::new();
		let mut key_to_id: HashMap<(i64, i64, i64), u32> = HashMap::new();
		let intern = |v: Vec3, verts: &mut Vec<Vec3>, map: &mut HashMap<(i64, i64, i64), u32>| -> u32 {
			let k = ((v.x as f64 / weld).round() as i64, (v.y as f64 / weld).round() as i64, (v.z as f64 / weld).round() as i64);
			*map.entry(k).or_insert_with(|| {
				let id = verts.len() as u32;
				verts.push(v);
				id
			})
		};
		let mut edges: Vec<(u32, u32)> = Vec::new();
		for s in &segs {
			let a = intern(s[0], &mut verts, &mut key_to_id);
			let b = intern(s[1], &mut verts, &mut key_to_id);
			if a != b {
				edges.push((a, b));
			}
		}

		// 3) Walk the edge set into rings (degree-2 at every interior vertex).
		let mut adj: Vec<Vec<(u32, usize)>> = vec![Vec::new(); verts.len()];
		for (ei, &(a, b)) in edges.iter().enumerate() {
			adj[a as usize].push((b, ei));
			adj[b as usize].push((a, ei));
		}
		let mut used = vec![false; edges.len()];
		let mut loops: Vec<Vec<Vec3>> = Vec::new();
		for e0 in 0..edges.len() {
			if used[e0] {
				continue;
			}
			used[e0] = true;
			let (start, b0) = edges[e0];
			let mut ring = vec![start];
			let mut cur = b0;
			loop {
				if cur == start {
					break; // closed
				}
				ring.push(cur);
				match adj[cur as usize].iter().copied().find(|&(_, ei)| !used[ei]) {
					Some((nb, ei)) => {
						used[ei] = true;
						cur = nb;
					}
					None => break, // open polyline
				}
			}
			if ring.len() >= 2 {
				let nd = n.as_dvec3();
				let mut area = 0.0f64;
				for k in 0..ring.len() {
					let a = verts[ring[k] as usize].as_dvec3();
					let c = verts[ring[(k + 1) % ring.len()] as usize].as_dvec3();
					area += a.cross(c).dot(nd);
				}
				let mut pts: Vec<Vec3> = ring.iter().map(|&i| verts[i as usize]).collect();
				if area < 0.0 {
					pts.reverse(); // wind CCW about +normal
				}
				loops.push(pts);
			}
		}
		loops
	}

	/// Classify each face for additive-manufacturing support against the upward
	/// `build_dir`. `support_overhang_deg` is the steepest overhang — measured from
	/// vertical — that still prints unsupported (a vertical wall is 0°, a horizontal
	/// ceiling is 90°; the common default is 45°). A face whose outward normal `n`
	/// satisfies `n·build_dir < −sin(support_overhang_deg)` overhangs too far and is
	/// flagged. Outward winding is assumed (call [`ensure_outward`](Self::ensure_outward)
	/// first if unsure). Larger angles are more permissive.
	pub fn overhang_analysis(&self, build_dir: Vec3, support_overhang_deg: f32) -> OverhangReport {
		let up = build_dir.normalize_or_zero().as_dvec3();
		// Same f64 threshold + measured slack as `support_free_report` — see the
		// long note there for why an f32-evaluated threshold mis-classified
		// geometry built exactly ON the limit angle, and why the slack is 1e-4.
		// The two reports must agree or a part can pass one audit and fail the
		// other.
		let threshold = -((support_overhang_deg as f64).to_radians().sin()) - 1e-4;
		let mut needs_support = Vec::with_capacity(self.triangle_count());
		let (mut overhang_area, mut total_area) = (0.0f64, 0.0f64);
		for t in self.indices.chunks_exact(3) {
			let a = self.positions[t[0] as usize].as_dvec3();
			let b = self.positions[t[1] as usize].as_dvec3();
			let c = self.positions[t[2] as usize].as_dvec3();
			let area_vec = (b - a).cross(c - a);
			let area = area_vec.length() * 0.5;
			total_area += area;
			let support = area_vec.normalize_or_zero().dot(up) < threshold;
			if support {
				overhang_area += area;
			}
			needs_support.push(support);
		}
		let overhang_fraction = if total_area > 0.0 { overhang_area / total_area } else { 0.0 };
		OverhangReport { overhang_area, total_area, overhang_fraction, needs_support }
	}

	/// FDM **support-necessity** audit of this mesh printed as-oriented with `build_dir`
	/// up: like [`overhang_analysis`](Self::overhang_analysis) but honest about what a
	/// printer actually needs. Downward faces past `support_overhang_deg` (from vertical;
	/// 45° is the common limit) are classified: triangles lying entirely within `bed_tol`
	/// of the lowest point are **bed contact** (first layer — always fine); ceilings
	/// within 1° of dead horizontal are **bridges** (printable; the report's
	/// `max_bridge_span` bounds the widest one via the AABB diagonal of each
	/// vertex-connected bridge patch); everything else flagged is **steep** and would
	/// need support. `steep_area == 0` ⟺ the part prints support-free in this
	/// orientation. Outward winding is assumed. `bed_tol` is in mesh units (mm by
	/// convention — a first-layer height, e.g. 0.3, is a good value).
	pub fn support_free_report(&self, build_dir: Vec3, support_overhang_deg: f32, bed_tol: f32) -> SupportFreeReport {
		let up = build_dir.normalize_or_zero().as_dvec3();
		// The threshold is evaluated in f64 and given a hair of slack, and both
		// halves of that matter (found 2026-07-30 by the API.md pass):
		//
		// * The angle used to be converted and sined in **f32** before widening,
		//   so at 45° the threshold was −0.7071067690849304 while a facet built
		//   at exactly 45° has the f64 normal −0.70710678118654746. The facet is
		//   1.2e-8 "steeper" than a threshold that is supposed to equal it, so
		//   geometry designed ON the limit was reported as needing support:
		//   `teardrop_hole` (roof at exactly 45°) measured 8 mm² of steep area at
		//   overhang_deg 45 and 0 at 46. Any part designed on its own threshold
		//   hit this, not just teardrops.
		// * A facet built at exactly the limit angle still cannot LAND there: mesh
		//   positions are f32, so its f64 normal carries the representation noise
		//   of its own coordinates. That noise was MEASURED, not guessed — over
		//   angles 30–60°, part scales 0–250 mm and facet edges 0.5–10 mm the
		//   worst cosine deviation is 1.14e-5 (it grows with |position| / edge
		//   length, so a small facet far from the origin is the bad case). The
		//   slack below sits ~9× above that.
		//
		// This is float noise, not a loosened requirement: 1e-4 in cosine is
		// 0.008° at 45° — no slicer, printer or designer distinguishes 45.000°
		// from 45.008°, while a facet genuinely past the limit (campaigns gate
		// `steep_area < 1e-6` mm²) still trips the gate. Pinned by
		// `kernel-core/tests/support_threshold.rs`, which asserts BOTH that an
		// at-threshold facet passes and that a 1°-past facet still fails.
		const SLACK_COS: f64 = 1e-4;
		let threshold = -((support_overhang_deg as f64).to_radians().sin()) - SLACK_COS;
		let flat = -(1.0f64.to_radians().cos()); // n·up below this ⇒ within 1° of a flat ceiling
		let zmin = self
			.positions
			.iter()
			.map(|p| p.as_dvec3().dot(up))
			.fold(f64::INFINITY, f64::min);
		let bed_z = zmin + bed_tol.max(0.0) as f64;

		// Union-find over vertex ids, joined only across bridge triangles, so each
		// connected ceiling patch can report its extent.
		let mut parent: Vec<u32> = (0..self.positions.len() as u32).collect();
		fn find(parent: &mut [u32], mut i: u32) -> u32 {
			while parent[i as usize] != i {
				parent[i as usize] = parent[parent[i as usize] as usize];
				i = parent[i as usize];
			}
			i
		}

		let mut steep = Vec::with_capacity(self.triangle_count());
		let (mut bed_area, mut bridge_area, mut steep_area, mut total_area) = (0.0f64, 0.0, 0.0, 0.0);
		let mut bridge_tris: Vec<[u32; 3]> = Vec::new();
		let mut steep_cands: Vec<(f64, crate::math::DVec3)> = Vec::new();
		for t in self.indices.chunks_exact(3) {
			let (a, b, c) = (
				self.positions[t[0] as usize].as_dvec3(),
				self.positions[t[1] as usize].as_dvec3(),
				self.positions[t[2] as usize].as_dvec3(),
			);
			let area_vec = (b - a).cross(c - a);
			let area = area_vec.length() * 0.5;
			total_area += area;
			let n_up = area_vec.normalize_or_zero().dot(up);
			let mut is_steep = false;
			if n_up < threshold {
				if a.dot(up) <= bed_z && b.dot(up) <= bed_z && c.dot(up) <= bed_z {
					bed_area += area;
				} else if n_up < flat {
					bridge_area += area;
					bridge_tris.push([t[0], t[1], t[2]]);
					let ra = find(&mut parent, t[0]);
					let rb = find(&mut parent, t[1]);
					let rc = find(&mut parent, t[2]);
					parent[rb as usize] = ra;
					parent[rc as usize] = ra;
				} else {
					steep_area += area;
					is_steep = true;
					steep_cands.push((area, (a + b + c) / 3.0));
				}
			}
			steep.push(is_steep);
		}
		steep_cands.sort_by(|x, y| y.0.partial_cmp(&x.0).unwrap_or(std::cmp::Ordering::Equal));
		let steep_exemplars: Vec<Vec3> = steep_cands.iter().take(8).map(|(_, c)| c.as_vec3()).collect();

		// True span per connected bridge patch: 2 × the deepest interior point's
		// distance to the patch BOUNDARY (patch edges used by only one patch
		// triangle). This is the span a printer actually bridges: a Ø10 disc
		// ceiling spans 10, a 300×8 slot spans 8, and an ANNULUS spans its radial
		// width — the previous min-projected-extent metric reported an annular
		// ring at its full outer diameter (no projection can see a hole).
		let (bu, bv) = {
			let helper = if up.x.abs() < 0.9 { crate::math::DVec3::X } else { crate::math::DVec3::Y };
			let u0 = up.cross(helper).normalize();
			(u0, up.cross(u0))
		};
		let uv = |vi: u32| {
			let p = self.positions[vi as usize].as_dvec3();
			(p.dot(bu), p.dot(bv))
		};
		// group patch triangles + count in-patch edge use
		let mut patch_tris: std::collections::HashMap<u32, Vec<[u32; 3]>> = std::collections::HashMap::new();
		for t in &bridge_tris {
			let root = find(&mut parent, t[0]);
			patch_tris.entry(root).or_default().push(*t);
		}
		let mut max_bridge_span = 0.0f64;
		let mut bridge_patches: Vec<(f64, Vec3)> = Vec::new();
		for tris in patch_tris.values() {
			let mut edge_use: std::collections::HashMap<(u32, u32), u32> = std::collections::HashMap::new();
			for t in tris {
				for k in 0..3 {
					let (a, b) = (t[k], t[(k + 1) % 3]);
					*edge_use.entry(if a < b { (a, b) } else { (b, a) }).or_insert(0) += 1;
				}
			}
			let boundary: Vec<((f64, f64), (f64, f64))> = edge_use
				.iter()
				.filter(|(_, &n)| n == 1)
				.map(|(&(a, b), _)| (uv(a), uv(b)))
				.collect();
			if boundary.is_empty() {
				continue;
			}
			let seg_dist = |p: (f64, f64), a: (f64, f64), b: (f64, f64)| -> f64 {
				let (dx, dy) = (b.0 - a.0, b.1 - a.1);
				let len2 = dx * dx + dy * dy;
				let t = if len2 > 1e-18 { (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len2).clamp(0.0, 1.0) } else { 0.0 };
				let (cx, cy) = (a.0 + t * dx - p.0, a.1 + t * dy - p.1);
				(cx * cx + cy * cy).sqrt()
			};
			// interior samples: triangle centroids + INTERIOR-edge midpoints (on a
			// coarse mesh the deep point often lies on a shared edge — e.g. the
			// diagonal midpoint of a two-triangle square ceiling). Each sample
			// keeps its 3D point so the patch can report WHERE it is.
			let p3 = |vi: u32| self.positions[vi as usize].as_dvec3();
			let mut samples: Vec<((f64, f64), crate::math::DVec3)> = tris
				.iter()
				.map(|t| {
					let (a, b, c) = (uv(t[0]), uv(t[1]), uv(t[2]));
					let m3 = (p3(t[0]) + p3(t[1]) + p3(t[2])) / 3.0;
					(((a.0 + b.0 + c.0) / 3.0, (a.1 + b.1 + c.1) / 3.0), m3)
				})
				.collect();
			for (&(a, b), &n) in &edge_use {
				if n >= 2 {
					let (pa, pb) = (uv(a), uv(b));
					samples.push((((pa.0 + pb.0) * 0.5, (pa.1 + pb.1) * 0.5), (p3(a) + p3(b)) * 0.5));
				}
			}
			let mut depth = 0.0f64;
			let mut deep_at = crate::math::DVec3::ZERO;
			for (s, s3) in samples {
				let d = boundary.iter().map(|&(pa, pb)| seg_dist(s, pa, pb)).fold(f64::INFINITY, f64::min);
				if d > depth {
					depth = d;
					deep_at = s3;
				}
			}
			max_bridge_span = max_bridge_span.max(2.0 * depth);
			bridge_patches.push((2.0 * depth, deep_at.as_vec3()));
		}
		bridge_patches.sort_by(|x, y| y.0.partial_cmp(&x.0).unwrap_or(std::cmp::Ordering::Equal));
		bridge_patches.truncate(8);

		SupportFreeReport {
			bed_area,
			bridge_area,
			steep_area,
			total_area,
			max_bridge_span,
			steep,
			steep_exemplars,
			bridge_patches,
		}
	}

	/// Moldability analysis against the mold `pull_dir` (the direction the mold
	/// opens). Each face's draft angle is `arcsin(|n·pull|)` — 0° for a wall
	/// parallel to pull (it would drag), 90° for a face square to pull. Faces below
	/// `min_draft_deg` are summed into `low_draft_area`. A face is an **undercut**
	/// (trapped — neither mold half can release it) when casting from just off its
	/// surface along *both* `+pull` and `−pull` hits more material; those are summed
	/// into `undercut_area`. Uses the BVH, so the pass is `O(n log n)`.
	pub fn draft_analysis(&self, pull_dir: Vec3, min_draft_deg: f32) -> DraftReport {
		let pull = pull_dir.normalize_or_zero();
		let bvh = self.build_bvh();
		let eps = self.aabb().size().length().max(1.0) * 1e-5;
		let mut draft_deg = Vec::with_capacity(self.triangle_count());
		let mut undercut = Vec::with_capacity(self.triangle_count());
		let (mut min_draft_deg_out, mut low_draft_area, mut undercut_area) = (f64::INFINITY, 0.0f64, 0.0f64);
		for t in self.indices.chunks_exact(3) {
			let a = self.positions[t[0] as usize];
			let b = self.positions[t[1] as usize];
			let c = self.positions[t[2] as usize];
			let area_vec = (b - a).cross(c - a);
			let area = (area_vec.length() * 0.5) as f64;
			let n = area_vec.normalize_or_zero();
			let draft = (n.dot(pull).abs().clamp(0.0, 1.0).asin().to_degrees()) as f64;
			draft_deg.push(draft);
			min_draft_deg_out = min_draft_deg_out.min(draft);
			if (draft as f32) < min_draft_deg {
				low_draft_area += area;
			}
			// Undercut: from just outside the surface, material lies along both pulls.
			let centroid = (a + b + c) / 3.0;
			let start = centroid + n * eps;
			let occluded_pos = bvh.raycast(Ray::new(start, pull)).is_some();
			let occluded_neg = bvh.raycast(Ray::new(start, -pull)).is_some();
			let uc = occluded_pos && occluded_neg;
			undercut.push(uc);
			if uc {
				undercut_area += area;
			}
		}
		DraftReport {
			min_draft_deg: if min_draft_deg_out.is_finite() { min_draft_deg_out } else { 0.0 },
			draft_deg,
			low_draft_area,
			undercut_area,
			undercut,
		}
	}

	/// Reduce the cross-section by the given plane to its [`SectionProperties`]
	/// (net area, perimeter, centroid, and second moments of area about the
	/// centroid). Holes are handled by even–odd nesting of the contour loops, so a
	/// tube section correctly subtracts its bore. Returns `None` if the plane
	/// misses the solid or the section is degenerate.
	pub fn section_properties(&self, point: Vec3, normal: Vec3) -> Option<SectionProperties> {
		let n = normal.normalize_or_zero();
		if n == Vec3::ZERO {
			return None;
		}
		let basis = if n.x.abs() < 0.9 { Vec3::X } else { Vec3::Y };
		let u = n.cross(basis).normalize_or_zero();
		let v = n.cross(u); // unit; u × v = n
		let loops = self.cross_section(point, n);
		if loops.is_empty() {
			return None;
		}
		// Project each loop into the (u, v) plane.
		let polys: Vec<Vec<Vec2>> = loops
			.iter()
			.map(|ring| ring.iter().map(|&p| Vec2::new(p.dot(u), p.dot(v))).collect())
			.collect();
		// Even–odd nesting: a loop contained in an odd number of others is a hole.
		let sign = |i: usize| -> f64 {
			let probe = polys[i][0];
			let depth = polys.iter().enumerate().filter(|&(j, p)| j != i && point_in_poly_2d(probe, p)).count();
			if depth % 2 == 0 {
				1.0
			} else {
				-1.0
			}
		};
		let (mut a, mut mx, mut my, mut sxx, mut syy, mut sxy, mut perim) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
		for (i, poly) in polys.iter().enumerate() {
			let (la, lmx, lmy, lsxx, lsyy, lsxy) = polygon_moments(poly);
			let s = sign(i);
			a += s * la;
			mx += s * lmx;
			my += s * lmy;
			sxx += s * lsxx;
			syy += s * lsyy;
			sxy += s * lsxy;
			for k in 0..poly.len() {
				perim += (poly[(k + 1) % poly.len()] - poly[k]).length() as f64;
			}
		}
		if a.abs() < 1e-12 {
			return None;
		}
		let (cu, cv) = (mx / a, my / a);
		let c_max = polys
			.iter()
			.flatten()
			.map(|p| (((p.x as f64) - cu).powi(2) + ((p.y as f64) - cv).powi(2)).sqrt())
			.fold(0.0f64, f64::max);
		let d = point.dot(n) as f64;
		Some(SectionProperties {
			area: a,
			perimeter: perim,
			centroid: u * cu as f32 + v * cv as f32 + n * d as f32,
			i_uu: sxx - a * cu * cu,
			i_vv: syy - a * cv * cv,
			i_uv: sxy - a * cu * cv,
			c_max,
			u_axis: u,
			v_axis: v,
		})
	}

	/// Bounding box of the vertices.
	pub fn aabb(&self) -> Aabb {
		Aabb::from_points(&self.positions)
	}

	/// Inertia-aligned oriented bounding box ([`Obb`]): orient the box by the
	/// solid's principal axes — so it is tight for a part that is simply rotated off
	/// the world axes (exact for a rotated box) — and size it by the extreme vertex
	/// projections onto those axes. Because the frame comes from the inertia tensor
	/// it is density-aware rather than biased by tessellation sampling. This is not
	/// the provably minimum-volume box (which is far costlier to compute), but it is
	/// typically far tighter than the [`aabb`](Self::aabb) for off-axis parts.
	pub fn oriented_bounding_box(&self) -> Obb {
		let axes = self.mass_properties().principal_axes().axes;
		let (e0, e1, e2) = (axes.x_axis, axes.y_axis, axes.z_axis);
		let (mut lo, mut hi) = (DVec3::splat(f64::INFINITY), DVec3::splat(f64::NEG_INFINITY));
		for &v in &self.positions {
			let p = v.as_dvec3();
			let q = DVec3::new(p.dot(e0), p.dot(e1), p.dot(e2));
			lo = lo.min(q);
			hi = hi.max(q);
		}
		if !lo.is_finite() {
			return Obb { center: DVec3::ZERO, axes, half_extents: DVec3::ZERO };
		}
		let mid = (hi + lo) * 0.5; // box center in the axis frame
		Obb {
			center: e0 * mid.x + e1 * mid.y + e2 * mid.z,
			axes,
			half_extents: (hi - lo) * 0.5,
		}
	}

	/// Nearest forward intersection of `ray` with the surface (Möller–Trumbore per
	/// triangle), or `None` if the ray misses. `RayHit::t` is in units of the ray
	/// direction's length — normalize the direction for a true distance. The
	/// fundamental picking primitive (e.g. click-to-select in a viewer).
	pub fn raycast(&self, ray: Ray) -> Option<RayHit> {
		let mut best: Option<RayHit> = None;
		for (ti, t) in self.indices.chunks_exact(3).enumerate() {
			let a = self.positions[t[0] as usize];
			let b = self.positions[t[1] as usize];
			let c = self.positions[t[2] as usize];
			if let Some((th, pt, nrm)) = ray_triangle(ray, a, b, c) {
				if best.is_none_or(|h| th < h.t) {
					best = Some(RayHit { t: th, point: pt, normal: nrm, triangle: ti });
				}
			}
		}
		best
	}

	/// Nearest point on the surface to `query` (per-triangle exact closest point),
	/// or `None` for an empty mesh. The primitive behind snapping, proximity and
	/// measurement.
	pub fn closest_point(&self, query: Vec3) -> Option<ClosestPoint> {
		let mut best: Option<ClosestPoint> = None;
		for (ti, t) in self.indices.chunks_exact(3).enumerate() {
			let a = self.positions[t[0] as usize];
			let b = self.positions[t[1] as usize];
			let c = self.positions[t[2] as usize];
			let cp = closest_point_on_triangle(query, a, b, c);
			let dist = (cp - query).length();
			if best.is_none_or(|x| dist < x.distance) {
				best = Some(ClosestPoint { point: cp, distance: dist, triangle: ti });
			}
		}
		best
	}

	pub fn area_weighted_normals(&self) -> Vec<Vec3> {
		let mut normals = vec![Vec3::ZERO; self.positions.len()];
		for t in self.indices.chunks_exact(3) {
			let (i0, i1, i2) = (t[0] as usize, t[1] as usize, t[2] as usize);
			let a = self.positions[i0];
			let b = self.positions[i1];
			let c = self.positions[i2];
			// Cross-product length is twice the triangle area → area weighting.
			let face = (b - a).cross(c - a);
			normals[i0] += face;
			normals[i1] += face;
			normals[i2] += face;
		}
		for n in normals.iter_mut() {
			*n = n.normalize_or_zero();
		}
		normals
	}

	/// Recompute and store area-weighted vertex normals.
	pub fn compute_normals(&mut self) {
		self.normals = self.area_weighted_normals();
	}

	/// Reverse triangle winding (and flip stored normals).
	pub fn reverse_winding(&mut self) {
		for t in self.indices.chunks_exact_mut(3) {
			t.swap(1, 2);
		}
		for n in self.normals.iter_mut() {
			*n = -*n;
		}
	}

	/// Ensure an outward (positive-volume) winding, flipping if needed.
	pub fn ensure_outward(&mut self) {
		if self.signed_volume() < 0.0 {
			self.reverse_winding();
		}
	}

	/// Check 2-manifold-ness: every undirected edge is shared by exactly two
	/// triangles. Returns the number of edges that are *not* shared by two
	/// triangles (`0` ⇒ a closed manifold surface).
	pub fn non_manifold_edge_count(&self) -> usize {
		let mut counts: HashMap<(u32, u32), u32> = HashMap::new();
		for t in self.indices.chunks_exact(3) {
			for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
				let key = if a < b { (a, b) } else { (b, a) };
				*counts.entry(key).or_insert(0) += 1;
			}
		}
		counts.values().filter(|&&c| c != 2).count()
	}

	/// True if every undirected edge is shared by exactly two triangles (edge
	/// closure: no boundary or non-manifold edges).
	///
	/// NOTE — this is NOT a full 2-manifold test: it does not catch non-orientable
	/// edges or pinched/bowtie (non-manifold) *vertices*, both of which keep every
	/// edge used twice. For the rigorous closed-orientable-2-manifold guarantee use
	/// [`Mesh::is_two_manifold`] (or [`crate::check_mesh`] for the full breakdown,
	/// which also reports self-intersection). Validity gates that must not ship a
	/// non-2-manifold solid should prefer those.
	pub fn is_watertight(&self) -> bool {
		!self.is_empty() && self.non_manifold_edge_count() == 0
	}

	/// How many **separate connected bodies** this mesh is in — union-find over
	/// position-welded vertices (`weld_tol` in model units; 1e-3 mm is the house
	/// value, matching `Mesh::weld`'s working scale).
	///
	/// `weld_tol` is a true TOLERANCE, not a grid pitch: any two vertices no
	/// farther apart than `weld_tol` are treated as one point, wherever the part
	/// sits in space. It therefore also sets the resolution of the oracle — a
	/// severance NARROWER than `weld_tol` is welded shut and reads as one body,
	/// which is why it is a caller-visible parameter and why `shells` (which
	/// counts B-rep records and sees a boolean severance at any width) is the
	/// COMPLEMENTARY check, not a weaker one.
	///
	/// # Why this is a THIRD oracle, not a restatement of the other two
	///
	/// Connectivity is independent of validity and of watertightness, and it is
	/// the one an engine cannot infer from either. A part severed into two lumps
	/// by a cut is:
	///
	/// * **valid** — each lump is a closed orientable solid, so `validate` is happy;
	/// * **watertight** — every edge is still used exactly twice;
	/// * **correctly measured** — `volume()` simply sums both lumps into a
	///   plausible number, and every clearance, stress and export gate downstream
	///   passes on that number;
	/// * and **`Solid::shell_count()` may still report 1**, because that counts
	///   B-rep shell RECORDS, not connected geometry.
	///
	/// This is not hypothetical. It was found on 2026-07-31 by a cold-start
	/// session building `hook_system/drill_hook`: an early draft ran a tapered
	/// cutter's apex out through both end faces, leaving the channel's front wall
	/// as a free-floating body. `validate`, `is_watertight`, `volume`, the
	/// support audit, the keep-out and insertion sweeps, the stress sections and
	/// the STEP round-trip ALL passed, and the render looked like a hook. Only a
	/// human noticing a gap in the top view caught it (campaign/friction/ENGINE.md).
	///
	/// **Operating rule**: any campaign that subtracts a tapered or tapering
	/// cutter must gate `component_count(1e-3) == 1`. A cutter whose
	/// cross-section shrinks to a point must keep that apex strictly INSIDE the
	/// material.
	///
	/// Returns 0 for an empty mesh. Cost is near-linear in triangle count while
	/// `weld_tol` is at or below the facet scale (each grid cell then holds a
	/// handful of vertices); a `weld_tol` MUCH coarser than the facets puts many
	/// vertices per cell and the within-cell pairing grows quadratically in that
	/// occupancy. Deterministic regardless: union-find connectivity does not
	/// depend on the order cells are visited.
	pub fn component_count(&self, weld_tol: f32) -> usize {
		if self.is_empty() {
			return 0;
		}
		// Vertices are the union-find nodes; the grid is only an ACCELERATOR for
		// finding which of them are within `weld_tol`. Making cell membership
		// itself the merge rule (what this used to do) turns `weld_tol` into a
		// grid PITCH rather than a tolerance: whether two points 0.4·weld_tol
		// apart share a cell depends on where they sit relative to the grid, so
		// the same gap in the same part answered 1 or 2 depending on the part's
		// absolute position in space (campaign theme T6). Testing the real
		// distance makes the contract the honest one — *any two vertices no
		// farther apart than `weld_tol` are the same point, wherever the part
		// sits* — at the cost of one distance test per near pair.
		let tol = if weld_tol > 0.0 { weld_tol as f64 } else { 1e-3 };
		let q = 1.0 / tol;
		let key = |p: &Vec3| -> (i64, i64, i64) {
			(
				(p.x as f64 * q).round() as i64,
				(p.y as f64 * q).round() as i64,
				(p.z as f64 * q).round() as i64,
			)
		};
		let mut cells: std::collections::HashMap<(i64, i64, i64), Vec<u32>> = std::collections::HashMap::new();
		for (i, p) in self.positions.iter().enumerate() {
			cells.entry(key(p)).or_default().push(i as u32);
		}
		let mut parent: Vec<u32> = (0..self.positions.len() as u32).collect();
		fn find(parent: &mut [u32], mut i: u32) -> u32 {
			while parent[i as usize] != i {
				parent[i as usize] = parent[parent[i as usize] as usize];
				i = parent[i as usize];
			}
			i
		}
		let tol2 = tol * tol;
		let unite = |parent: &mut Vec<u32>, a: u32, b: u32| {
			let (ra, rb) = (find(parent, a), find(parent, b));
			if ra != rb {
				parent[rb as usize] = ra;
			}
		};
		// Cells are `weld_tol` wide, so a vertex within the tolerance is either in
		// this cell or in one of the 26 neighbours — the sweep stays near-linear
		// while `weld_tol` is at or below the facet scale (1e-3 mm by default).
		let cell_keys: Vec<(i64, i64, i64)> = cells.keys().copied().collect();
		let near = |i: u32, j: u32| -> bool {
			(self.positions[i as usize].as_dvec3() - self.positions[j as usize].as_dvec3()).length_squared() <= tol2
		};
		for k in &cell_keys {
			let here = cells[k].clone();
			for a in 0..here.len() {
				for b in (a + 1)..here.len() {
					if near(here[a], here[b]) {
						unite(&mut parent, here[a], here[b]);
					}
				}
			}
			for dz in -1i64..=1 {
				for dy in -1i64..=1 {
					for dx in -1i64..=1 {
						// Half the neighbourhood: every unordered cell pair is visited
						// exactly once, from its lexicographically lower key.
						if (dx, dy, dz) <= (0, 0, 0) {
							continue;
						}
						let Some(there) = cells.get(&(k.0 + dx, k.1 + dy, k.2 + dz)) else {
							continue;
						};
						for &i in &here {
							for &j in there {
								if near(i, j) {
									unite(&mut parent, i, j);
								}
							}
						}
					}
				}
			}
		}
		for t in self.indices.chunks_exact(3) {
			unite(&mut parent, t[0], t[1]);
			unite(&mut parent, t[0], t[2]);
		}
		// Count roots of vertices that a triangle actually references; an
		// unreferenced stray position is not a body.
		let mut used = vec![false; parent.len()];
		for &i in &self.indices {
			used[i as usize] = true;
		}
		let mut roots = std::collections::HashSet::new();
		for i in 0..parent.len() as u32 {
			if used[i as usize] {
				let r = find(&mut parent, i);
				roots.insert(r);
			}
		}
		roots.len()
	}

	/// `component_count` at the house weld scale (1e-3 model units) — the form a
	/// campaign gate should call: `gate("one body", m.is_one_body(), …)`.
	pub fn is_one_body(&self) -> bool {
		self.component_count(1e-3) == 1
	}

	/// True if the mesh is a closed, orientable 2-manifold: no boundary, no
	/// non-manifold or non-orientable edges, and no non-manifold (bowtie) vertices.
	/// The rigorous companion to [`Mesh::is_watertight`] (which only checks edge
	/// closure). Does not test self-intersection — see [`crate::check_mesh`].
	pub fn is_two_manifold(&self) -> bool {
		crate::meshcheck::is_two_manifold(self)
	}

	/// Fill boundary loops (holes) with a centroid-fan cap, making an open mesh
	/// watertight; returns the number of holes filled. A *boundary edge* is a
	/// directed edge whose reverse is absent; these chain into closed loops, each
	/// capped by a fan to its own centroid (so non-planar holes are handled too).
	/// The caps reuse each boundary edge's reverse, so the result is closed. Run
	/// [`weld`](Self::weld) first if vertices are per-face-duplicated, so boundary
	/// edges actually share endpoints. The primary repair for imported / scanned
	/// meshes before meshing-dependent queries (mass properties, booleans).
	///
	/// At a *pinch* vertex shared by two boundary loops the greedy walk may splice
	/// them into one cap (every edge is still consumed and the mesh is still closed,
	/// but the cap is not the ideal two separate fans); turn-aware loop separation is
	/// a possible refinement. The walk order — including that splice choice — is
	/// deterministic (boundary edges are taken in triangle order), so identical
	/// input always yields the identical repaired mesh.
	pub fn fill_holes(&mut self) -> usize {
		// Directed-edge counts for O(1) edge-use lookups, plus the same edges in
		// first-insertion (triangle) order. Boundary edges must NOT be collected by
		// iterating a hash container: its order is seeded per instance, and that
		// order decides each loop's start vertex, the cap emission order, and — at a
		// pinch vertex on two hole rims — which loop the walk splices into, so
		// identical input was repaired differently run to run.
		let (dir, ordered) = self.directed_edges();
		// Boundary edges: undirected edges used by exactly ONE triangle, in triangle
		// order. Keep them as a list and a tail→edge multimap, so a vertex that is
		// the tail of two boundary edges (a pinch where two holes meet) does not
		// drop one of them.
		//
		// "Used once" and not "the reverse `b→a` is absent": two triangles that wind
		// the same way over an edge leave no rim to cap, and fanning one anyway adds
		// a THIRD triangle to an edge that already had two — turning a winding defect
		// into a non-manifold one. Same rule as [`Mesh::boundary_edge_count`], which
		// is the measure this repair is supposed to drive to zero.
		let boundary: Vec<(u32, u32)> =
			ordered.into_iter().filter(|&(a, b)| Self::is_boundary_edge(&dir, a, b)).collect();
		if boundary.is_empty() {
			return 0;
		}
		let mut by_tail: HashMap<u32, Vec<usize>> = HashMap::new();
		for (i, &(a, _)) in boundary.iter().enumerate() {
			by_tail.entry(a).or_default().push(i);
		}
		let mut used = vec![false; boundary.len()];
		let mut filled = 0;
		for start_e in 0..boundary.len() {
			if used[start_e] {
				continue;
			}
			// Walk a loop by consuming one unused boundary edge per step.
			let mut ring: Vec<u32> = Vec::new();
			let mut e = start_e;
			loop {
				used[e] = true;
				let (a, b) = boundary[e];
				ring.push(a);
				match by_tail.get(&b).and_then(|es| es.iter().copied().find(|&j| !used[j])) {
					Some(j) => e = j,
					None => break, // loop closed (b == ring[0]) or dead-ended
				}
			}
			if ring.len() < 3 {
				continue;
			}
			let mut centroid = Vec3::ZERO;
			for &i in &ring {
				centroid += self.positions[i as usize];
			}
			let ci = self.push_vertex(centroid / ring.len() as f32);
			// Cap each boundary edge `a→b` with `(ci, b, a)`, contributing the `b→a`
			// that the boundary lacked; the fan spokes pair among themselves.
			for k in 0..ring.len() {
				let a = ring[k];
				let b = ring[(k + 1) % ring.len()];
				self.push_triangle(ci, b, a);
			}
			filled += 1;
		}
		filled
	}

	/// Repair an imported / scanned triangle mesh toward a watertight solid: [`weld`](Self::weld)
	/// coincident vertices (recovering shared edges from an STL-style triangle soup) then
	/// [`fill_holes`] to close boundary loops. The canonical one-call cleanup before treating a
	/// mesh read via [`read_stl`](Self::read_stl) / `read_obj` / `read_ply` / `read_3mf` as a
	/// solid. NOTE: this fixes soup seams and boundary holes — the common import defects; it does
	/// **not** resolve the saddle pinches of a thin TPMS/lattice shell (an inherently non-solid
	/// surface — heal-to-manifold is not possible there). For non-manifold input, run
	/// [`make_manifold`](crate::make_manifold) first.
	///
	/// [`fill_holes`]: Self::fill_holes
	pub fn make_watertight(&self) -> Mesh {
		let mut m = self.clone();
		let tol = m.aabb().size().length().max(1.0) * 1e-5;
		m.weld(tol);
		m.fill_holes();
		m
	}

	/// Merge vertices that coincide within `tolerance` (true Euclidean distance)
	/// and remap the triangle indices, so that per-face-duplicated vertices (e.g.
	/// from B-rep tessellation) become a shared-vertex manifold mesh. Normals of
	/// merged vertices are averaged.
	///
	/// Uses a spatial hash at cell size `tolerance` and probes the 27-cell
	/// neighbourhood, so a pair within `tolerance` is merged regardless of which
	/// cell each lands in. Keys are computed in `f64` to stay exact at large
	/// coordinate magnitudes.
	pub fn weld(&mut self, tolerance: f32) {
		if self.positions.is_empty() {
			return;
		}
		let tol = tolerance.max(1e-9);
		let tol2 = tol * tol;
		let inv = 1.0_f64 / tol as f64;
		let key = |p: Vec3| -> (i64, i64, i64) {
			(
				(p.x as f64 * inv).round() as i64,
				(p.y as f64 * inv).round() as i64,
				(p.z as f64 * inv).round() as i64,
			)
		};
		let has_normals = self.normals.len() == self.positions.len();
		// Each cell holds the new-vertex ids that were created there.
		let mut map: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
		let mut new_pos: Vec<Vec3> = Vec::new();
		let mut new_nrm: Vec<Vec3> = Vec::new();
		let mut remap = vec![0u32; self.positions.len()];
		for (old, &p) in self.positions.iter().enumerate() {
			let k = key(p);
			// Look for an existing representative within `tolerance` in the 27
			// cells around this one.
			let mut found: Option<u32> = None;
			'search: for dz in -1..=1 {
				for dy in -1..=1 {
					for dx in -1..=1 {
						if let Some(reps) = map.get(&(k.0 + dx, k.1 + dy, k.2 + dz)) {
							for &r in reps {
								if (new_pos[r as usize] - p).length_squared() <= tol2 {
									found = Some(r);
									break 'search;
								}
							}
						}
					}
				}
			}
			let id = match found {
				Some(r) => r,
				None => {
					let id = new_pos.len() as u32;
					new_pos.push(p);
					if has_normals {
						new_nrm.push(Vec3::ZERO);
					}
					map.entry(k).or_default().push(id);
					id
				}
			};
			if has_normals {
				new_nrm[id as usize] += self.normals[old];
			}
			remap[old] = id;
		}
		for idx in self.indices.iter_mut() {
			*idx = remap[*idx as usize];
		}
		// Drop triangles the weld collapsed (two corners merged into one vertex).
		// Such a needle's two long edges are the SAME segment, so keeping it
		// double-counts that edge and a closed surface stops reading watertight —
		// a boolean-recovery zero-area sliver face meshed exactly this way.
		let mut kept = Vec::with_capacity(self.indices.len());
		for t in self.indices.chunks_exact(3) {
			if t[0] != t[1] && t[1] != t[2] && t[2] != t[0] {
				kept.extend_from_slice(t);
			}
		}
		self.indices = kept;
		if has_normals {
			for n in new_nrm.iter_mut() {
				*n = n.normalize_or_zero();
			}
			self.normals = new_nrm;
		}
		self.positions = new_pos;
	}

	/// **Taubin (λ|μ) smoothing**: `iterations` passes of a low-pass filter that relaxes
	/// the staircase faceting/aliasing of a voxel- or dual-contoured surface *without* the
	/// shrinkage of plain Laplacian smoothing (each λ-shrink pass is followed by a μ-inflate
	/// pass). Operates on the shared (indexed) one-ring, so [`weld`](Self::weld) the mesh
	/// first if its vertices are split per-triangle. A few iterations turn a lumpy voxel
	/// shank into a smooth one while keeping feature ridges (e.g. thread crests). Normals
	/// are invalidated and recomputed.
	pub fn taubin_smooth(&mut self, iterations: usize) {
		if self.positions.is_empty() || self.indices.len() < 3 {
			return;
		}
		let lambda = 0.5_f32;
		let mu = -0.53_f32;
		// One-ring adjacency from the triangle edges.
		let n = self.positions.len();
		let mut nbrs: Vec<Vec<u32>> = vec![Vec::new(); n];
		for t in self.indices.chunks_exact(3) {
			for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
				if !nbrs[a as usize].contains(&b) {
					nbrs[a as usize].push(b);
				}
				if !nbrs[b as usize].contains(&a) {
					nbrs[b as usize].push(a);
				}
			}
		}
		let relax = |positions: &mut [Vec3], factor: f32| {
			let avg: Vec<Vec3> = (0..positions.len())
				.map(|i| {
					if nbrs[i].is_empty() {
						positions[i]
					} else {
						let s: Vec3 = nbrs[i].iter().fold(Vec3::ZERO, |acc, &j| acc + positions[j as usize]);
						s / nbrs[i].len() as f32
					}
				})
				.collect();
			for (p, a) in positions.iter_mut().zip(avg.iter()) {
				*p += (*a - *p) * factor;
			}
		};
		for _ in 0..iterations {
			relax(&mut self.positions, lambda);
			relax(&mut self.positions, mu);
		}
		self.compute_normals();
	}

	/// Vertex-clustering decimation (Rossignac–Borrel): merge vertices within
	/// `cell_size`, drop the triangles that collapse to a degenerate (repeated
	/// index), and recompute normals. A fast, robust level-of-detail reducer for
	/// preview / export. The result is lower-poly and is *not* guaranteed
	/// manifold (clustering can merge nearby sheets) — run [`make_manifold`] or
	/// re-mesh finer if a manifold is required.
	///
	/// [`make_manifold`]: crate::make_manifold
	pub fn decimate(&self, cell_size: f32) -> Mesh {
		let mut m = self.clone();
		m.weld(cell_size);
		let mut kept = Vec::with_capacity(m.indices.len());
		for t in m.indices.chunks_exact(3) {
			if t[0] != t[1] && t[1] != t[2] && t[0] != t[2] {
				kept.extend_from_slice(t);
			}
		}
		m.indices = kept;
		m.compute_normals();
		m
	}

	// --- Importers -----------------------------------------------------------

}
#[cfg(test)]
mod tests {
	use super::formats::*;
	use super::*;

	#[test]
	fn fill_holes_reported_announces_the_geometry_it_invents() {
		// A closed unit cube with its TOP face removed — an open box with one
		// 4-edge square hole. Plain fill_holes returns a bare "1"; the reported
		// variant must ANNOUNCE that re-closing it invents an interior (the volume
		// jumps from the open surface's ~0 toward the sealed cube), names the
		// largest opening (4 edges), and clears the boundary.
		let v = [
			Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 0.0), Vec3::new(0.0, 1.0, 0.0),
			Vec3::new(0.0, 0.0, 1.0), Vec3::new(1.0, 0.0, 1.0), Vec3::new(1.0, 1.0, 1.0), Vec3::new(0.0, 1.0, 1.0),
		];
		let mut m = Mesh::new();
		for p in v {
			m.positions.push(p);
		}
		// five faces (bottom + four sides), TOP (4,5,6,7) left open, outward-wound
		for t in [
			[0, 2, 1], [0, 3, 2],           // bottom (-z)
			[0, 1, 5], [0, 5, 4],           // front (-y)
			[1, 2, 6], [1, 6, 5],           // right (+x)
			[2, 3, 7], [2, 7, 6],           // back (+y)
			[3, 0, 4], [3, 4, 7],           // left (-x)
		] {
			m.push_triangle(t[0], t[1], t[2]);
		}
		let open_edges = m.boundary_edge_count();
		let vol_open = m.signed_volume().abs();
		let d = m.fill_holes_reported();
		assert!(
			d.op == "fill_holes"
				&& d.holes_filled == 1
				&& d.largest_opening_edges == 4
				&& d.open_edges_before == open_edges && open_edges == 4
				&& d.open_edges_after == 0
				&& d.changed_geometry()
				&& d.volume_delta().abs() > 0.1
				&& d.triangles_after > d.triangles_before,
			"fill_holes_reported must ANNOUNCE the repair: got {d:?} (open surface |vol| was {vol_open:.3})"
		);
	}

	#[test]
	fn write_then_read_3mf_round_trips_a_mesh() {
		// 3MF ingestion — the reading half of 3MF interchange (only writing existed). Write a
		// known tetrahedron to a real OPC 3MF package and read it back: the vertex positions
		// and triangle indices must come back identical, proving the zip + `.model` + vertex/
		// triangle XML parse round-trips. (f32 Display is round-trip exact for these values.)
		let mut src = Mesh::new();
		for p in [Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)] {
			src.positions.push(p);
		}
		for t in [[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]] {
			src.push_triangle(t[0], t[1], t[2]);
		}
		let path = std::env::temp_dir().join(format!("kernel_3mf_roundtrip_{}.3mf", std::process::id()));
		src.write_3mf(&path).expect("write 3mf");
		let back = Mesh::read_3mf(&path).expect("read 3mf");
		let _ = std::fs::remove_file(&path);
		assert!(
			back.positions == src.positions && back.indices == src.indices,
			"3MF round-trip mismatch: {} verts / {} tris back vs {} / {}",
			back.positions.len(),
			back.triangle_count(),
			src.positions.len(),
			src.triangle_count()
		);
	}

	#[test]
	fn parse_3mf_mesh_is_attribute_order_and_noise_tolerant() {
		// A model fragment as a THIRD-PARTY 3MF writer might emit it: indented with newlines,
		// vertex attributes in a different order (z, y, x), extra material attributes on the
		// triangle (p1/pid), and the <vertices>/<triangles> wrappers — which must NOT be
		// mistaken for <vertex>/<triangle>. Exercises the parser the round-trip can't.
		let xml = "<mesh>\n  <vertices>\n    <vertex z=\"2.5\" y=\"0\" x=\"1\"/>\n    <vertex x=\"0\" y=\"0\" z=\"0\"/>\n  </vertices>\n  <triangles>\n    <triangle v1=\"0\" v2=\"1\" v3=\"0\" p1=\"0\" pid=\"5\"/>\n  </triangles>\n</mesh>";
		let verts = xml_elements(xml, "vertex");
		let tris = xml_elements(xml, "triangle");
		assert!(
			verts.len() == 2
				&& xml_attr_f32(verts[0], "x") == Some(1.0)
				&& xml_attr_f32(verts[0], "z") == Some(2.5)
				&& tris.len() == 1
				&& xml_attr_u32(tris[0], "v2") == Some(1),
			"tolerant 3MF parse failed: {} verts (want 2), {} tris (want 1)",
			verts.len(),
			tris.len()
		);
	}

	#[test]
	fn make_watertight_repairs_a_holed_mesh() {
		// A closed tetrahedron with one face removed leaves a triangular hole — the kind of
		// defect a scanned / imported mesh carries. make_watertight (weld → separate → fill)
		// must close it back into a watertight 2-manifold. (A thin TPMS shell can't be — its
		// non-manifold saddle pinches aren't boundary holes; documented on the method.)
		let mut tetra = Mesh::new();
		for p in [Vec3::ZERO, Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0), Vec3::new(0.0, 0.0, 1.0)] {
			tetra.positions.push(p);
		}
		// Three of the four consistently-wound faces of a closed tetra (the (1,3,2) face omitted
		// leaves a single triangular boundary loop 1→2→3).
		for f in [[0, 1, 2], [0, 3, 1], [0, 2, 3]] {
			tetra.push_triangle(f[0], f[1], f[2]);
		}
		let healed = tetra.make_watertight();
		assert!(
			!tetra.is_watertight() && healed.is_watertight(),
			"make_watertight must close the hole: before={} after={} ({} tris)",
			tetra.is_watertight(),
			healed.is_watertight(),
			healed.triangle_count()
		);
	}

	#[test]
	fn from_ply_reads_ascii_vertices_and_a_fan_triangulated_quad() {
		// ASCII PLY ingestion: a square with an EXTRA per-vertex property (`nx`, value 9 — must
		// be IGNORED, only x/y/z taken) and a single quad face that fan-triangulates into two
		// triangles. Proves header-count parsing, property-skipping, and face fan handling.
		let ply = "ply\nformat ascii 1.0\nelement vertex 4\nproperty float x\nproperty float y\nproperty float z\nproperty float nx\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n0 0 0 9\n1 0 0 9\n1 1 0 9\n0 1 0 9\n4 0 1 2 3\n";
		let m = Mesh::from_ply_bytes(ply.as_bytes()).expect("parse PLY");
		assert!(
			m.positions.len() == 4 && m.triangle_count() == 2 && (m.positions[2] - Vec3::new(1.0, 1.0, 0.0)).length() < 1e-6,
			"PLY must parse 4 verts / 2 tris (quad fan) with v2=(1,1,0): verts={} tris={}",
			m.positions.len(),
			m.triangle_count()
		);
	}

	#[test]
	fn from_obj_reads_vertices_and_mixed_face_forms() {
		// OBJ ingestion — the reading half of OBJ interchange (only writing existed before). A unit
		// tetrahedron with a comment, an ignored `vn`, one face in the `a//vn` form write_obj emits
		// and three in the bare `a b c` form parses to exactly 4 vertices / 4 triangles, with the
		// last vertex at (0,0,1) — proving v-parsing, slash-index handling, and fan/face indexing.
		let obj = "# unit tetra\nv 0 0 0\nv 1 0 0\nv 0 1 0\nv 0 0 1\nvn 0 0 1\nf 1//1 2//1 3//1\nf 1 2 4\nf 1 3 4\nf 2 3 4\n";
		let m = Mesh::from_obj_bytes(obj.as_bytes()).expect("parse OBJ");
		assert!(
			m.positions.len() == 4 && m.triangle_count() == 4 && (m.positions[3] - Vec3::new(0.0, 0.0, 1.0)).length() < 1e-6,
			"OBJ must parse to 4 verts / 4 tris with v3=(0,0,1): verts={} tris={} v3={:?}",
			m.positions.len(),
			m.triangle_count(),
			m.positions.get(3)
		);
	}

	#[test]
	fn taubin_smooth_preserves_watertight_topology() {
		// Smoothing relaxes vertex positions but never changes connectivity, so a closed
		// watertight octahedron stays watertight with the same triangle count and a
		// still-positive volume (Taubin does not collapse the mesh the way plain Laplacian
		// shrinkage would).
		let mut m = Mesh::new();
		for p in [
			Vec3::new(5.0, 0.0, 0.0),
			Vec3::new(-5.0, 0.0, 0.0),
			Vec3::new(0.0, 5.0, 0.0),
			Vec3::new(0.0, -5.0, 0.0),
			Vec3::new(0.0, 0.0, 5.0),
			Vec3::new(0.0, 0.0, -5.0),
		] {
			m.push_vertex(p);
		}
		for t in [[0, 2, 4], [2, 1, 4], [1, 3, 4], [3, 0, 4], [2, 0, 5], [1, 2, 5], [3, 1, 5], [0, 3, 5]] {
			m.push_triangle(t[0], t[1], t[2]);
		}
		assert!(m.is_watertight(), "octahedron should start watertight");
		let tris = m.triangle_count();
		m.taubin_smooth(3);
		assert!(
			m.is_watertight() && m.triangle_count() == tris && m.signed_volume().abs() > 0.5,
			"taubin must preserve watertight topology: wt={} tris={} vol={}",
			m.is_watertight(),
			m.triangle_count(),
			m.signed_volume()
		);
	}

	#[test]
	fn self_intersection_detects_crossing_triangles() {
		// Two far-apart triangles do not self-intersect; two crossing, non-adjacent
		// triangles do — the geometric validity check that topology alone can't catch.
		let mut clean = Mesh::new();
		clean.positions = vec![
			Vec3::new(0.0, 0.0, 0.0),
			Vec3::new(1.0, 0.0, 0.0),
			Vec3::new(0.0, 1.0, 0.0),
			Vec3::new(5.0, 0.0, 0.0),
			Vec3::new(6.0, 0.0, 0.0),
			Vec3::new(5.0, 1.0, 0.0),
		];
		clean.indices = vec![0, 1, 2, 3, 4, 5];

		let mut crossing = Mesh::new();
		crossing.positions = vec![
			// a horizontal triangle in z = 0 around the origin
			Vec3::new(-1.0, -1.0, 0.0),
			Vec3::new(1.0, -1.0, 0.0),
			Vec3::new(0.0, 1.0, 0.0),
			// a vertical triangle (y = 0 plane) passing through z = 0 inside the first
			Vec3::new(-0.5, 0.0, -1.0),
			Vec3::new(0.5, 0.0, -1.0),
			Vec3::new(0.0, 0.0, 1.0),
		];
		crossing.indices = vec![0, 1, 2, 3, 4, 5];

		assert!(
			!clean.has_self_intersection() && crossing.has_self_intersection(),
			"disjoint pair clean={}, crossing pair flagged={}",
			clean.has_self_intersection(),
			crossing.has_self_intersection()
		);
	}
}

