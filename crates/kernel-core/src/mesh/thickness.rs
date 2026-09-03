// Copyright (c) LMCAD. Licensed under the MIT License.

//! Ray-based wall thickness — the DFM probe behind the `wall_thickness` op.
//!
//! From a surface sample, cast a ray inward (along `−normal`) and read the
//! distance to the opposite wall: the local material thickness under that
//! point. `thin_area` sums the surface area whose reading is below the
//! caller's `flag_below`.
//!
//! # Sampling: area-uniform, stratified, deterministic
//!
//! One centroid ray per triangle (the historical sampler) makes `thin_area` a
//! triangulation lottery: a boolean leaves a face split into a few huge
//! triangles and a handful of slivers, so whether a thin patch is "seen" depends
//! on where the centroids happen to land — mirror-image bodies read 5× apart
//! (friction l12_mini_case F4, 2026-09). This sampler instead spreads a fixed
//! budget ([`THICKNESS_SAMPLE_BUDGET`]) of samples uniformly over the SURFACE
//! AREA: every triangle larger than one budget cell is split into `m²`
//! congruent barycentric sub-triangles (`m = ⌈√(area / cell)⌉`) and one sample is
//! taken inside each, jittered by a hash of `(triangle, sub-cell)` so the
//! estimate is unbiased instead of lattice-aligned. Each sample carries its
//! sub-cell's area as weight, so the weights sum to the mesh area exactly. A
//! triangle at or below one cell (a fine voxel-route mesh) keeps the single
//! centroid sample, byte-for-byte the historical reading. No RNG state: the same
//! mesh always yields the same samples, and two triangulations of the same
//! surface agree to the sampling noise (≈1 % of a thin band's area at the
//! default budget), not to the luck of the diagonals.
//!
//! # Acute wedges (knife-edge readings)
//!
//! Where two faces meet at a convex material angle below 90° — the lip of a
//! female dovetail, the rim of a cone base, a bevel run out to a point — the
//! material under the surface next to that edge is genuinely thin, and every
//! ray from that lip band exits through the NEIGHBOURING face. Those readings
//! are physical, but they are edge geometry, not a wall, and they consume a
//! `thin_area: 0` gate no design can pass. [`ThicknessOptions::exclude_wedge_deg`]
//! sets such readings aside: a sample whose ray exits through a face that
//! shares an edge with the sample's own face, at a convex material dihedral
//! below the threshold, is counted under `thin_area_wedge` instead of
//! `thin_area`. "Face" here is a group of edge-connected triangles with
//! near-parallel normals (a planar face, or a coarse facet of a curved one),
//! so the rule is about the B-rep faces, not the triangulation. Two PARALLEL
//! faces (a thin plate, a drafted wall) never share an edge, so a real thin wall
//! is never a wedge; a concave notch is never a wedge either (the convexity test
//! uses the far corner of the neighbouring triangle).

use std::collections::HashMap;

use super::{Mesh, ThicknessReport};
use crate::math::{DVec3, Ray, Vec3};
use crate::meshcheck::UnionFind;

/// Target number of stratified surface samples per thickness pass. The budget
/// is spread over the surface area; triangles smaller than one budget cell still
/// get their one centroid sample, so a fine mesh may exceed it.
pub const THICKNESS_SAMPLE_BUDGET: usize = 65_536;

/// Upper bound on the per-side subdivision of one triangle (`m` in the module
/// doc): a single triangle never takes more than `MAX_SIDE²` samples.
const MAX_SIDE: usize = 256;

/// Two triangles whose unit normals agree to within this dot product belong to
/// the same face for the wedge rule (≈ 1°, wide enough to absorb the normal
/// noise of boolean slivers, narrow enough that any real wedge is far below).
const COPLANAR_DOT: f64 = 0.999_85;

/// Controls for [`Mesh::wall_thickness_with`].
#[derive(Clone, Copy, Debug)]
pub struct ThicknessOptions {
	/// Readings below this thickness are flagged (summed into the thin areas).
	pub flag_below: f64,
	/// When set, flagged readings whose ray exits through an edge-adjacent face
	/// at a convex material dihedral below this many degrees are counted under
	/// `thin_area_wedge` instead of `thin_area` (see the module doc).
	pub exclude_wedge_deg: Option<f64>,
}

/// One surface sample of a thickness pass.
#[derive(Clone, Copy, Debug)]
pub struct ThicknessSample {
	/// The sampled surface point (the ray origin before its inward offset).
	pub point: Vec3,
	/// Material thickness under `point`; [`f64::INFINITY`] when the inward ray
	/// escaped (an open mesh or a through-hole).
	pub thickness: f64,
	/// Surface area this sample stands for (its stratum), in model units².
	pub area: f64,
	/// Index of the triangle the sample lies on.
	pub triangle: usize,
	/// `true` when the reading is an acute-wedge reading under the requested
	/// exclusion (always `false` without one).
	pub wedge: bool,
}

impl Mesh {
	/// Ray-based wall-thickness analysis with no wedge exclusion — see
	/// [`Mesh::wall_thickness_with`] and the [module doc](self).
	pub fn wall_thickness(&self, flag_below: f64) -> ThicknessReport {
		self.wall_thickness_with(ThicknessOptions { flag_below, exclude_wedge_deg: None })
	}

	/// Ray-based wall-thickness analysis: area-uniform stratified samples over
	/// every face, one inward ray each, the flagged area split into wall
	/// readings (`thin_area`) and, when `exclude_wedge_deg` is set, knife-edge
	/// readings (`thin_area_wedge`). Uses the [`MeshBvh`](crate::MeshBvh)
	/// internally, so the pass is `O((n + samples) log n)`. Outward winding is
	/// assumed; an inward ray that escapes records [`f64::INFINITY`].
	pub fn wall_thickness_with(&self, opts: ThicknessOptions) -> ThicknessReport {
		let bvh = self.build_bvh();
		let eps = self.aabb().size().length().max(1.0) * 1e-5;
		let tri_count = self.triangle_count();

		let mut corners: Vec<[Vec3; 3]> = Vec::with_capacity(tri_count);
		let mut normals: Vec<Vec3> = Vec::with_capacity(tri_count);
		let mut areas: Vec<f64> = Vec::with_capacity(tri_count);
		for t in self.indices.chunks_exact(3) {
			let a = self.positions[t[0] as usize];
			let b = self.positions[t[1] as usize];
			let c = self.positions[t[2] as usize];
			let area_vec = (b - a).cross(c - a);
			corners.push([a, b, c]);
			normals.push(area_vec.normalize_or_zero());
			areas.push((area_vec.length() * 0.5) as f64);
		}
		let total_area: f64 = areas.iter().sum();
		let cell = total_area / THICKNESS_SAMPLE_BUDGET as f64;
		let wedge = opts.exclude_wedge_deg.map(|deg| WedgeOracle::new(self, &normals, deg));

		let mut thickness = Vec::with_capacity(tri_count);
		let mut samples: Vec<ThicknessSample> = Vec::with_capacity(tri_count.max(THICKNESS_SAMPLE_BUDGET));
		let mut min_thickness = f64::INFINITY;
		let (mut thin_area, mut thin_area_wedge) = (0.0f64, 0.0f64);

		for ti in 0..tri_count {
			let [a, b, c] = corners[ti];
			let normal = normals[ti];
			let area = areas[ti];
			// Start a hair inside so the originating face is behind the ray.
			let probe = |p: Vec3| -> (f64, Option<usize>) {
				match bvh.raycast(Ray::new(p - normal * eps, -normal)) {
					Some(hit) => ((hit.t + eps) as f64, Some(hit.triangle)),
					None => (f64::INFINITY, None),
				}
			};
			let mut record = |point: Vec3, th: f64, hit: Option<usize>, weight: f64| {
				let is_wedge = match (&wedge, hit) {
					(Some(w), Some(h)) => w.is_wedge(ti, h, &normals),
					_ => false,
				};
				samples.push(ThicknessSample { point, thickness: th, area: weight, triangle: ti, wedge: is_wedge });
				if is_wedge {
					if th < opts.flag_below {
						thin_area_wedge += weight;
					}
				} else {
					min_thickness = min_thickness.min(th);
					if th < opts.flag_below {
						thin_area += weight;
					}
				}
			};

			let centroid = (a + b + c) / 3.0;
			let (centroid_th, centroid_hit) = probe(centroid);
			thickness.push(centroid_th);

			let side = if cell > 0.0 && area > cell { ((area / cell).sqrt().ceil() as usize).clamp(2, MAX_SIDE) } else { 1 };
			if side == 1 {
				record(centroid, centroid_th, centroid_hit, area);
				continue;
			}
			// `side²` congruent sub-triangles on the barycentric lattice, one
			// jittered sample each.
			let (a64, ab, ac) = (a.as_dvec3(), (b - a).as_dvec3(), (c - a).as_dvec3());
			let lattice = |i: usize, j: usize| a64 + ab * (i as f64 / side as f64) + ac * (j as f64 / side as f64);
			let sub_area = area / (side * side) as f64;
			let mut k = 0u64;
			for i in 0..side {
				for j in 0..side - i {
					let up = [lattice(i, j), lattice(i + 1, j), lattice(i, j + 1)];
					let p = jittered_point(up, ti as u64, k);
					k += 1;
					let (th, hit) = probe(p);
					record(p, th, hit, sub_area);
					if i + j + 2 <= side {
						let down = [lattice(i + 1, j), lattice(i + 1, j + 1), lattice(i, j + 1)];
						let p = jittered_point(down, ti as u64, k);
						k += 1;
						let (th, hit) = probe(p);
						record(p, th, hit, sub_area);
					}
				}
			}
		}
		ThicknessReport { min_thickness, thickness, thin_area, thin_area_wedge, samples, exclude_wedge_deg: opts.exclude_wedge_deg }
	}
}

/// A point uniformly distributed inside the sub-triangle `tri`, chosen by a
/// hash of `(triangle, sub-cell)` — deterministic, seedless, and independent of
/// every other sub-cell.
fn jittered_point(tri: [DVec3; 3], triangle: u64, cell: u64) -> Vec3 {
	let (mut u, mut v) = unit_pair(triangle.wrapping_mul(0x1_0000_0000).wrapping_add(cell));
	if u + v > 1.0 {
		u = 1.0 - u;
		v = 1.0 - v;
	}
	(tri[0] + (tri[1] - tri[0]) * u + (tri[2] - tri[0]) * v).as_vec3()
}

/// Two uniform variates in `[0, 1)` from a 64-bit seed (splitmix64).
fn unit_pair(seed: u64) -> (f64, f64) {
	let mut state = seed;
	let mut next = move || {
		state = state.wrapping_add(0x9E37_79B9_7F4A_7C15);
		let mut z = state;
		z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
		z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
		z ^= z >> 31;
		(z >> 11) as f64 / (1u64 << 53) as f64
	};
	(next(), next())
}

/// The face-adjacency oracle behind the wedge exclusion (module doc).
struct WedgeOracle {
	/// Face group (union-find root) per triangle.
	group: Vec<usize>,
	/// Edge-adjacent face-group pairs `(lo, hi)` → `true` when the shared edge
	/// is convex (the neighbour's far corner lies on the material side).
	adjacent: HashMap<(usize, usize), bool>,
	/// `cos(exclude_wedge_deg)`: a pair is a wedge when the cosine of its
	/// material dihedral (`−n₁·n₂`) exceeds this.
	cos_limit: f64,
}

impl WedgeOracle {
	fn new(mesh: &Mesh, normals: &[Vec3], limit_deg: f64) -> Self {
		let tri_count = normals.len();
		// Undirected edge → the first two triangles using it + use count.
		let mut by_edge: HashMap<(u32, u32), (usize, usize, u32)> = HashMap::with_capacity(tri_count * 3 / 2);
		for (ti, t) in mesh.indices.chunks_exact(3).enumerate() {
			for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
				let key = if a < b { (a, b) } else { (b, a) };
				let e = by_edge.entry(key).or_insert((ti, usize::MAX, 0));
				if e.2 == 1 {
					e.1 = ti;
				}
				e.2 += 1;
			}
		}
		let degenerate = |t: usize| normals[t].length_squared() < 0.5;
		let mut uf = UnionFind::new(tri_count);
		let mut pairs: Vec<(usize, usize, (u32, u32))> = Vec::new();
		let mut through_slivers: Vec<(usize, Vec<usize>)> = Vec::new();
		for (&edge, &(t0, t1, uses)) in &by_edge {
			if uses != 2 {
				continue; // rim or fin: no face adjacency claimed across it
			}
			if degenerate(t0) || degenerate(t1) {
				continue; // handled below: a zero-area sliver is transparent
			}
			if normals[t0].as_dvec3().dot(normals[t1].as_dvec3()) > COPLANAR_DOT {
				uf.union(t0, t1);
			} else {
				pairs.push((t0, t1, edge));
			}
		}
		// A zero-area sliver (a boolean's T-junction filler on a shared edge)
		// separates the faces on either side of it topologically; treat it as
		// transparent so those faces still count as meeting.
		for ti in (0..tri_count).filter(|&t| degenerate(t)) {
			let t = &mesh.indices[3 * ti..3 * ti + 3];
			let mut neighbours = Vec::new();
			for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
				let key = if a < b { (a, b) } else { (b, a) };
				if let Some(&(t0, t1, 2)) = by_edge.get(&key) {
					let other = if t0 == ti { t1 } else { t0 };
					if !degenerate(other) {
						neighbours.push(other);
					}
				}
			}
			through_slivers.push((ti, neighbours));
		}
		let group: Vec<usize> = (0..tri_count).map(|t| uf.find(t)).collect();
		let ordered = |x: usize, y: usize| if x < y { (x, y) } else { (y, x) };
		let mut adjacent: HashMap<(usize, usize), bool> = HashMap::new();
		for &(t0, t1, (ea, eb)) in &pairs {
			let (g0, g1) = (group[t0], group[t1]);
			if g0 == g1 {
				continue;
			}
			// Convex iff the neighbour's corner off the shared edge lies on the
			// material side (below) of this triangle's plane.
			let far = mesh.indices[3 * t1..3 * t1 + 3].iter().find(|&&v| v != ea && v != eb).copied();
			let convex = far.is_some_and(|v| {
				let on_edge = mesh.positions[ea as usize].as_dvec3();
				(mesh.positions[v as usize].as_dvec3() - on_edge).dot(normals[t0].as_dvec3()) < 0.0
			});
			let e = adjacent.entry(ordered(g0, g1)).or_insert(convex);
			*e |= convex;
		}
		for (_, neighbours) in &through_slivers {
			for (i, &x) in neighbours.iter().enumerate() {
				for &y in &neighbours[i + 1..] {
					let (gx, gy) = (group[x], group[y]);
					if gx != gy {
						adjacent.entry(ordered(gx, gy)).or_insert(true);
					}
				}
			}
		}
		let cos_limit = limit_deg.to_radians().cos();
		WedgeOracle { group, adjacent, cos_limit }
	}

	/// Is a reading taken on triangle `from` that exits through triangle `hit`
	/// an acute-wedge reading?
	fn is_wedge(&self, from: usize, hit: usize, normals: &[Vec3]) -> bool {
		let (gf, gh) = (self.group[from], self.group[hit]);
		if gf == gh {
			return false;
		}
		let key = if gf < gh { (gf, gh) } else { (gh, gf) };
		if !matches!(self.adjacent.get(&key), Some(true)) {
			return false;
		}
		let cos_material = -(normals[from].as_dvec3().dot(normals[hit].as_dvec3()));
		cos_material > self.cos_limit
	}
}

#[cfg(test)]
mod tests {
	use super::*;

	/// A closed box `[lo, hi]` with outward winding, one quad (two triangles) per
	/// face — the big-triangle case the stratified sampler exists for.
	fn boxed(lo: Vec3, hi: Vec3) -> Mesh {
		let mut m = Mesh::new();
		let p = |x: f32, y: f32, z: f32| Vec3::new(x, y, z);
		m.positions = vec![
			p(lo.x, lo.y, lo.z),
			p(hi.x, lo.y, lo.z),
			p(hi.x, hi.y, lo.z),
			p(lo.x, hi.y, lo.z),
			p(lo.x, lo.y, hi.z),
			p(hi.x, lo.y, hi.z),
			p(hi.x, hi.y, hi.z),
			p(lo.x, hi.y, hi.z),
		];
		let quads: [[u32; 4]; 6] = [
			[0, 3, 2, 1], // bottom (−z)
			[4, 5, 6, 7], // top (+z)
			[0, 1, 5, 4], // −y
			[2, 3, 7, 6], // +y
			[0, 4, 7, 3], // −x
			[1, 2, 6, 5], // +x
		];
		for q in quads {
			m.push_triangle(q[0], q[1], q[2]);
			m.push_triangle(q[0], q[2], q[3]);
		}
		m
	}

	/// The weights of the stratified samples sum to the surface area exactly,
	/// and a plate's two big faces are flagged in full: area-uniform sampling
	/// reproduces the historical `thin_area = 800` with no triangulation luck.
	#[test]
	fn stratified_samples_partition_the_surface_area() {
		let m = boxed(Vec3::new(-10.0, -10.0, 0.0), Vec3::new(10.0, 10.0, 0.5));
		let r = m.wall_thickness(1.0);
		let weight: f64 = r.samples.iter().map(|s| s.area).sum();
		assert!((weight - m.surface_area()).abs() < 1e-3, "sample weights {weight} must sum to the area {}", m.surface_area());
		assert!(r.samples.len() > 10_000, "a 12-triangle plate must be sampled far beyond one ray per triangle ({})", r.samples.len());
		assert_eq!(r.thickness.len(), m.triangle_count(), "per-triangle readings stay one per triangle");
		assert!((r.thin_area - 800.0).abs() < 1e-3, "thin_area {} must be the two 20×20 faces", r.thin_area);
		assert!((r.min_thickness - 0.5).abs() < 1e-4, "min_thickness {}", r.min_thickness);
		assert_eq!(r.thin_area_wedge, 0.0);
		assert!(r.samples.iter().all(|s| !s.wedge));
	}

	/// The sampler is a pure function of the mesh: two passes are identical.
	#[test]
	fn sampling_is_deterministic() {
		let m = boxed(Vec3::new(0.0, 0.0, 0.0), Vec3::new(30.0, 20.0, 10.0));
		let (a, b) = (m.wall_thickness(12.0), m.wall_thickness(12.0));
		assert_eq!(a.samples.len(), b.samples.len());
		assert!(a.samples.iter().zip(&b.samples).all(|(x, y)| x.point == y.point && x.thickness == y.thickness));
		assert_eq!(a.thin_area, b.thin_area);
	}

	/// A 30° wedge prism: the readings next to its apex are knife-edge readings.
	/// With the exclusion they move to `thin_area_wedge`; without it they are
	/// plain thin area — and the exclusion never touches the two parallel-ish
	/// far faces or the base.
	#[test]
	fn acute_wedge_readings_are_set_aside_only_when_asked() {
		// Cross-section in XZ: apex at the origin, base z = 20 from x = 0 to x = 20·tan30°.
		let w = 20.0f32 * 30f32.to_radians().tan();
		let mut m = Mesh::new();
		let p = |x: f32, y: f32, z: f32| Vec3::new(x, y, z);
		m.positions = vec![p(0.0, 0.0, 0.0), p(0.0, 0.0, 20.0), p(w, 0.0, 20.0), p(0.0, 30.0, 0.0), p(0.0, 30.0, 20.0), p(w, 30.0, 20.0)];
		// Outward winding: end caps, the x = 0 face, the base (z = 20), the slope.
		m.push_triangle(0, 2, 1);
		m.push_triangle(3, 4, 5);
		m.push_triangle(0, 1, 4);
		m.push_triangle(0, 4, 3);
		m.push_triangle(1, 2, 5);
		m.push_triangle(1, 5, 4);
		m.push_triangle(0, 3, 5);
		m.push_triangle(0, 5, 2);
		assert!(m.is_watertight());
		assert!(m.signed_volume() > 0.0, "outward winding");

		let plain = m.wall_thickness(2.0);
		let split = m.wall_thickness_with(ThicknessOptions { flag_below: 2.0, exclude_wedge_deg: Some(75.0) });
		assert!(plain.thin_area > 50.0, "the apex band must read thin without the exclusion: {}", plain.thin_area);
		assert_eq!(plain.thin_area_wedge, 0.0);
		assert!(
			split.thin_area < 1e-6 && split.thin_area_wedge > 50.0,
			"with the exclusion the apex band is wedge area, not wall area: thin={} wedge={}",
			split.thin_area,
			split.thin_area_wedge
		);
		let total = split.thin_area + split.thin_area_wedge;
		assert!((total - plain.thin_area).abs() < 1e-6, "the split must conserve the flagged area: {total} vs {}", plain.thin_area);
		assert!(split.min_thickness > 2.0, "min_thickness over the counted samples ignores the apex: {}", split.min_thickness);

		// A plain 1 mm plate never becomes a wedge: its faces are parallel.
		let plate = boxed(Vec3::new(0.0, 0.0, 0.0), Vec3::new(20.0, 20.0, 1.0));
		let r = plate.wall_thickness_with(ThicknessOptions { flag_below: 2.0, exclude_wedge_deg: Some(75.0) });
		assert!((r.thin_area - 800.0).abs() < 1e-3 && r.thin_area_wedge == 0.0, "plate: thin={} wedge={}", r.thin_area, r.thin_area_wedge);
	}
}
