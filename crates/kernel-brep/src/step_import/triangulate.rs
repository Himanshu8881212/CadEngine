// Copyright (c) LMCAD. Licensed under the MIT License.

//! Two parameter-space polygon triangulators: the general ear clip for a simple
//! ring, and the monotone sweep the periodic routes prefer because it never emits
//! a triangle that jumps a seam gap.

use kernel_core::math::DVec2;
use kernel_core::orient2d;

/// Ear-clip a SIMPLE (non-self-intersecting) uv polygon into index triangles
/// wound like the input loop. The general fallback for boundaries the monotone
/// sweep refuses (e.g. a half-cylinder band whose rim carries a lug notch —
/// unwrapped, that rim dips in v and is not u-monotone). Diagonals of a simple
/// polygon stay inside it, so the periodic-seam guarantee of the sweep is
/// preserved: no triangle can jump the seam gap.
pub(crate) fn triangulate_earclip(uv: &[DVec2]) -> Result<Vec<[usize; 3]>, String> {
	let n = uv.len();
	if n < 3 {
		return Err("fewer than three boundary points".into());
	}
	let area2: f64 = (0..n).map(|i| uv[i].x * uv[(i + 1) % n].y - uv[(i + 1) % n].x * uv[i].y).sum();
	if area2 == 0.0 {
		return Err("zero-area parameter-space boundary".into());
	}
	let ccw = area2 > 0.0;
	let o = |a: DVec2, b: DVec2, c: DVec2| orient2d([a.x, a.y], [b.x, b.y], [c.x, c.y]);
	let inside = |p: DVec2, a: DVec2, b: DVec2, c: DVec2| {
		let (d1, d2, d3) = (o(p, a, b), o(p, b, c), o(p, c, a));
		let has_neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
		let has_pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
		!(has_neg && has_pos)
	};
	let mut idx: Vec<usize> = if ccw { (0..n).collect() } else { (0..n).rev().collect() };
	let mut out: Vec<[usize; 3]> = Vec::with_capacity(n - 2);
	let mut guard = 0usize;
	while idx.len() > 3 && guard < 40_000 {
		guard += 1;
		let m = idx.len();
		let mut clipped = false;
		for i in 0..m {
			let (ip, ic, inx) = (idx[(i + m - 1) % m], idx[i], idx[(i + 1) % m]);
			let (a, b, c) = (uv[ip], uv[ic], uv[inx]);
			if o(a, b, c) <= 0.0 {
				continue; // reflex or flat corner
			}
			if idx.iter().any(|&j| j != ip && j != ic && j != inx && inside(uv[j], a, b, c)) {
				continue;
			}
			out.push(if ccw { [ip, ic, inx] } else { [inx, ic, ip] });
			idx.remove(i);
			clipped = true;
			break;
		}
		if !clipped {
			return Err("ear clipping stalled on a degenerate uv boundary".into());
		}
	}
	if idx.len() == 3 {
		out.push(if ccw { [idx[0], idx[1], idx[2]] } else { [idx[2], idx[1], idx[0]] });
	}
	Ok(out)
}

/// Triangulate a simple u-monotone polygon (`uv` in loop order) into index triangles
/// wound like the input loop, via the classic two-chain sweep with a reflex stack
/// (de Berg et al. §3.3). Every triangle connects u-adjacent vertices, so the two
/// sides of a periodic seam (which sit a full period apart with ring vertices between
/// them) never join one triangle. Errors with a reason on non-monotone or degenerate
/// input rather than emitting garbage.
pub(super) fn triangulate_monotone(uv: &[DVec2]) -> Result<Vec<[usize; 3]>, String> {
	let n = uv.len();
	if n < 3 {
		return Err("fewer than three boundary points".into());
	}
	let area2: f64 = (0..n)
		.map(|i| {
			let a = uv[i];
			let b = uv[(i + 1) % n];
			a.x * b.y - b.x * a.y
		})
		.sum();
	if area2 == 0.0 {
		return Err("zero-area parameter-space boundary".into());
	}
	// Work on a CCW view of the loop; flip the output winding back at the end.
	let ccw = area2 > 0.0;
	let at = |k: usize| if ccw { k } else { n - 1 - k };
	let p = |k: usize| uv[at(k)];
	let lex_less = |a: usize, b: usize| {
		let (pa, pb) = (p(a), p(b));
		pa.x.total_cmp(&pb.x).then(pa.y.total_cmp(&pb.y)) == std::cmp::Ordering::Less
	};
	let (mut lo, mut hi) = (0usize, 0usize);
	for k in 1..n {
		if lex_less(k, lo) {
			lo = k;
		}
		if lex_less(hi, k) {
			hi = k;
		}
	}
	// Chains: walking forward from the (lexicographic) min to the max is the LOWER
	// chain of a CCW polygon; walking backward, the upper. Verify monotonicity.
	let mut lower = vec![false; n];
	{
		let mut k = lo;
		while k != hi {
			lower[k] = true;
			let next = (k + 1) % n;
			if lex_less(next, k) {
				return Err("boundary is not u-monotone".into());
			}
			k = next;
		}
		let mut k = hi;
		while k != lo {
			let next = (k + 1) % n;
			if k != hi && lex_less(k, next) {
				return Err("boundary is not u-monotone".into());
			}
			k = next;
		}
	}
	// Merge the two (sorted) chains into one sweep order.
	let mut order: Vec<usize> = Vec::with_capacity(n);
	order.push(lo);
	{
		let (mut a, mut b) = ((lo + 1) % n, (lo + n - 1) % n);
		while a != hi || b != hi {
			if b == hi || (a != hi && lex_less(a, b)) {
				order.push(a);
				a = (a + 1) % n;
			} else {
				order.push(b);
				b = (b + n - 1) % n;
			}
		}
	}
	order.push(hi);
	for w in order.windows(2) {
		let (pa, pb) = (p(w[0]), p(w[1]));
		if pa.x == pb.x && pa.y == pb.y {
			return Err("coincident parameter-space boundary points".into());
		}
	}
	// Sweep with the reflex stack; orient every emitted triangle CCW (the pop order
	// alone does not fix the winding).
	let orient = |i: usize, j: usize, k: usize| orient2d([p(i).x, p(i).y], [p(j).x, p(j).y], [p(k).x, p(k).y]);
	let mut tris: Vec<[usize; 3]> = Vec::with_capacity(n - 2);
	let emit = |a: usize, b: usize, c: usize, tris: &mut Vec<[usize; 3]>| -> Result<(), String> {
		let o = orient(a, b, c);
		if o == 0.0 {
			return Err("degenerate (zero-height) parameter region".into());
		}
		tris.push(if o > 0.0 { [a, b, c] } else { [a, c, b] });
		Ok(())
	};
	let mut stack: Vec<usize> = vec![order[0], order[1]];
	for &vj in &order[2..n - 1] {
		let top = *stack.last().expect("the sweep stack is never empty");
		if lower[vj] != lower[top] {
			// Opposite chain: vj sees the whole stack; fan across it.
			while stack.len() > 1 {
				let v1 = stack.pop().expect("len > 1");
				let v2 = *stack.last().expect("len > 1 before pop");
				emit(vj, v1, v2, &mut tris)?;
			}
			stack.pop();
			stack.push(top);
		} else {
			// Same chain: clip while the diagonal to the next stack vertex stays inside
			// (a left turn seen from the lower chain, a right turn from the upper).
			let mut v1 = stack.pop().expect("the sweep stack is never empty");
			while let Some(&v2) = stack.last() {
				let o = orient(v2, v1, vj);
				let inside = if lower[vj] { o > 0.0 } else { o < 0.0 };
				if !inside {
					break;
				}
				emit(vj, v1, v2, &mut tris)?;
				v1 = v2;
				stack.pop();
			}
			stack.push(v1);
		}
		stack.push(vj);
	}
	// The final (max) vertex closes out every remaining stack diagonal.
	while stack.len() > 1 {
		let v1 = stack.pop().expect("len > 1");
		let v2 = *stack.last().expect("len > 1 before pop");
		emit(hi, v1, v2, &mut tris)?;
	}
	if tris.len() != n - 2 {
		return Err(format!("triangulation produced {} triangles for a {n}-gon", tris.len()));
	}
	Ok(tris
		.into_iter()
		.map(|t| {
			let m = [at(t[0]), at(t[1]), at(t[2])];
			if ccw {
				m
			} else {
				[m[0], m[2], m[1]]
			}
		})
		.collect())
}
