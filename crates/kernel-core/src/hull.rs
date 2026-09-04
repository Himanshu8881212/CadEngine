// Copyright (c) LMCAD. Licensed under the MIT License.

//! The 3-D convex hull of a point set, by the incremental algorithm: seed a
//! tetrahedron from four affinely-independent extremes, then fold each remaining
//! point in by deleting the faces it can see and capping the resulting horizon.
//! Useful for collision proxies, a minimal convex enclosure, GJK support meshes
//! and nesting. The result is a closed, outward-oriented [`Mesh`].

use std::collections::HashMap;
use std::collections::HashSet;

use crate::math::{Aabb, Vec3};
use crate::mesh::Mesh;

impl Mesh {
	/// The convex hull of this mesh's vertices, as a closed outward-oriented mesh.
	pub fn convex_hull(&self) -> Mesh {
		convex_hull(&self.positions)
	}
}

/// The convex hull of `points`. Returns an empty mesh for fewer than four points
/// or a degenerate (collinear / coplanar) set, which has no 3-D hull.
pub fn convex_hull(points: &[Vec3]) -> Mesh {
	if points.len() < 4 {
		return Mesh::new();
	}
	let eps = Aabb::from_points(points).size().length().max(1.0) * 1e-6;
	let Some([i0, i1, i2, i3]) = initial_tetra(points, eps) else {
		return Mesh::new();
	};
	let interior = (points[i0] + points[i1] + points[i2] + points[i3]) / 4.0;
	let mut faces: Vec<[usize; 3]> =
		[[i0, i1, i2], [i0, i1, i3], [i0, i2, i3], [i1, i2, i3]].into_iter().map(|f| orient_outward(points, f, interior)).collect();

	for p in 0..points.len() {
		if p == i0 || p == i1 || p == i2 || p == i3 {
			continue;
		}
		add_point(&mut faces, points, p, eps);
	}
	let hull = build_mesh(points, &faces);
	// Robustness contract: return a closed outward hull, or an empty mesh — never a
	// corrupt one. For sliver / near-coplanar clouds the incremental fold can leave
	// a non-manifold soup (the face normals lose precision); detect that and fall
	// back to empty (the documented degenerate result) rather than silently emitting
	// a broken mesh that downstream volume / BVH / watertight checks would trust.
	if hull.non_manifold_edge_count() != 0 {
		return Mesh::new();
	}
	hull
}

/// Twice-area face normal (un-normalized), pointing by the winding.
fn face_normal(points: &[Vec3], f: [usize; 3]) -> Vec3 {
	(points[f[1]] - points[f[0]]).cross(points[f[2]] - points[f[0]])
}

/// Flip `f` if needed so its normal points away from the `interior` point.
fn orient_outward(points: &[Vec3], f: [usize; 3], interior: Vec3) -> [usize; 3] {
	if face_normal(points, f).dot(interior - points[f[0]]) > 0.0 {
		[f[0], f[2], f[1]]
	} else {
		f
	}
}

/// Fold point `p` into the hull: delete every face it can see and cap the horizon.
fn add_point(faces: &mut Vec<[usize; 3]>, points: &[Vec3], p: usize, eps: f32) {
	let pp = points[p];
	let visible: Vec<usize> = faces
		.iter()
		.enumerate()
		.filter(|(_, &f)| {
			let n = face_normal(points, f);
			// Signed distance of `pp` above the face plane: n·(pp−v0) / |n|.
			n.dot(pp - points[f[0]]) > eps * n.length()
		})
		.map(|(i, _)| i)
		.collect();
	if visible.is_empty() {
		return; // `p` is inside the current hull
	}
	let vis: HashSet<usize> = visible.iter().copied().collect();

	// Horizon: each directed edge of a visible face whose reverse is not an edge of
	// another visible face — i.e. the border with the kept (non-visible) faces.
	let mut horizon: Vec<(usize, usize)> = Vec::new();
	for &fi in &visible {
		let f = faces[fi];
		for (a, b) in [(f[0], f[1]), (f[1], f[2]), (f[2], f[0])] {
			let shared = visible.iter().any(|&gi| {
				let g = faces[gi];
				gi != fi && [(g[0], g[1]), (g[1], g[2]), (g[2], g[0])].contains(&(b, a))
			});
			if !shared {
				horizon.push((a, b));
			}
		}
	}

	// Drop visible faces, then cap each horizon edge with a triangle to `p`. The
	// kept edge direction (a→b from the deleted visible face) gives the new face
	// the same outward winding as the rest of the hull.
	let kept: Vec<[usize; 3]> = faces.iter().enumerate().filter(|(i, _)| !vis.contains(i)).map(|(_, &f)| f).collect();
	*faces = kept;
	for (a, b) in horizon {
		faces.push([a, b, p]);
	}
}

/// Four affinely-independent points (extreme along a direction, then farthest
/// from the growing simplex), or `None` if the set is collinear / coplanar.
fn initial_tetra(points: &[Vec3], eps: f32) -> Option<[usize; 4]> {
	let n = points.len();
	let i0 = 0usize;
	let i1 = (0..n).max_by(|&a, &b| points[a].distance(points[i0]).total_cmp(&points[b].distance(points[i0])))?;
	if points[i1].distance(points[i0]) < eps {
		return None; // all coincident
	}
	let line_dist = |q: Vec3| ((q - points[i0]).cross(points[i1] - points[i0])).length() / (points[i1] - points[i0]).length();
	let i2 = (0..n).max_by(|&a, &b| line_dist(points[a]).total_cmp(&line_dist(points[b])))?;
	if line_dist(points[i2]) < eps {
		return None; // collinear
	}
	let nrm = (points[i1] - points[i0]).cross(points[i2] - points[i0]);
	let plane_dist = |q: Vec3| nrm.dot(q - points[i0]).abs() / nrm.length();
	let i3 = (0..n).max_by(|&a, &b| plane_dist(points[a]).total_cmp(&plane_dist(points[b])))?;
	if plane_dist(points[i3]) < eps {
		return None; // coplanar
	}
	Some([i0, i1, i2, i3])
}

/// Re-index the hull faces into a compact [`Mesh`].
fn build_mesh(points: &[Vec3], faces: &[[usize; 3]]) -> Mesh {
	let mut mesh = Mesh::new();
	let mut remap: HashMap<usize, u32> = HashMap::new();
	for f in faces {
		let mut tri = [0u32; 3];
		for (k, &v) in f.iter().enumerate() {
			tri[k] = *remap.entry(v).or_insert_with(|| {
				let id = mesh.positions.len() as u32;
				mesh.positions.push(points[v]);
				id
			});
		}
		mesh.push_triangle(tri[0], tri[1], tri[2]);
	}
	mesh
}
