// Copyright (c) LMCAD. Licensed under the MIT License.

//! Shared lattice tables for the dual meshers (Surface Nets and Dual Contouring).
//!
//! Both meshers march the same regular cell lattice and need the same cube-edge
//! topology, so the tables live here once rather than being duplicated per mesher.

/// Corner offsets of a unit cell, indexed by the bit pattern `x | y<<1 | z<<2`.
pub const CORNER_OFFSET: [[usize; 3]; 8] = [
	[0, 0, 0],
	[1, 0, 0],
	[0, 1, 0],
	[1, 1, 0],
	[0, 0, 1],
	[1, 0, 1],
	[0, 1, 1],
	[1, 1, 1],
];

/// Build the 12-edge table and the 256-entry per-cell edge-crossing mask.
///
/// `cube_edges[2k]`, `cube_edges[2k+1]` are the two corner indices of edge `k`.
/// Corner index `c` has offset [`CORNER_OFFSET`]`[c]`. Edges `0,1,2` are the +x,
/// +y, +z edges emanating from corner 0 — relied upon by quad/face emission.
/// `edge_table[mask]` has bit `k` set when edge `k` has a sign change for the
/// given inside/outside corner `mask`.
pub fn edge_tables() -> ([usize; 24], [u32; 256]) {
	let mut cube_edges = [0usize; 24];
	let mut k = 0;
	for i in 0..8usize {
		let mut j = 1usize;
		while j <= 4 {
			let p = i ^ j;
			if i <= p {
				cube_edges[k] = i;
				cube_edges[k + 1] = p;
				k += 2;
			}
			j <<= 1;
		}
	}
	let mut edge_table = [0u32; 256];
	for (mask, slot) in edge_table.iter_mut().enumerate() {
		let mut em = 0u32;
		let mut e = 0;
		while e < 24 {
			let a = (mask & (1 << cube_edges[e])) != 0;
			let b = (mask & (1 << cube_edges[e + 1])) != 0;
			if a != b {
				em |= 1 << (e >> 1);
			}
			e += 2;
		}
		*slot = em;
	}
	(cube_edges, edge_table)
}
