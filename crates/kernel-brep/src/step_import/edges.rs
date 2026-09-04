// Copyright (c) LMCAD. Licensed under the MIT License.

//! Edge-level reconstruction shared by every face route: how far a conic arc may
//! sweep before it is chorded, how a B-spline edge curve is sampled, and the small
//! position/ring helpers the loop readers share.

use kernel_core::math::DVec3;

use crate::nurbs::NurbsCurve;

use super::parse::{Entity, Value};
use super::StepError;

/// Segments a full 2π conic ring is tessellated into on import (~7.5° per chord).
pub(super) const FULL_TURN_SEGMENTS: usize = 48;

/// Largest conic-arc sweep (radians) imported as a single chord between its two
/// vertices. Up to here the producer's own edge granularity is respected — which also
/// keeps a re-import of this kernel's faceted exports bit-identical (round-trip volume
/// preserved to 1e-6). Beyond it (through full 2π rings, whose endpoints alone cannot
/// describe the boundary at all) the arc is subdivided at the `FULL_TURN_SEGMENTS`
/// pitch so the boundary is geometrically faithful.
pub(super) const MAX_CHORD_SWEEP: f64 = std::f64::consts::FRAC_PI_2;

/// Baseline segments a B-spline edge curve is sampled into across its knot domain
/// (doubled adaptively up to [`MAX_BSPLINE_EDGE_SEGMENTS`] while consecutive chords
/// turn by more than the conic ring pitch — see `edge_polyline`).
pub(super) const BSPLINE_EDGE_SEGMENTS: usize = 8;

/// Hard cap on adaptive B-spline edge sampling (a full rational circle terminates
/// at 64 segments — 5.6° per chord, finer than the 48-segment conic pitch).
pub(super) const MAX_BSPLINE_EDGE_SEGMENTS: usize = 64;

/// Largest turn angle (radians) between consecutive chords of an `n`-segment uniform
/// parameter sampling of a B-spline curve — the curvature witness that drives the
/// adaptive edge pitch. Near-zero chords (repeated control points) are skipped.
pub(super) fn max_chord_turn(c: &NurbsCurve, n: usize) -> f64 {
	let (lo, hi) = c.domain();
	let pts: Vec<DVec3> = (0..=n).map(|k| c.point_at(lo + (hi - lo) * k as f64 / n as f64)).collect();
	let scale = 1.0 + pts.iter().map(|p| p.length()).fold(0.0_f64, f64::max);
	let mut chords: Vec<DVec3> = Vec::with_capacity(n);
	for w in pts.windows(2) {
		let d = w[1] - w[0];
		if d.length() > 1e-12 * scale {
			chords.push(d.normalize());
		}
	}
	let mut turn = 0.0_f64;
	for w in chords.windows(2) {
		turn = turn.max(w[0].dot(w[1]).clamp(-1.0, 1.0).acos());
	}
	turn
}

/// The signed sweep of a conic edge from angle `t0` to `t1`: in `(0, 2π]` when the edge
/// follows the curve's parameterisation (`same_sense`), in `[−2π, 0)` against it.
/// Identical endpoint angles mean a FULL ring (sweep ±2π), per the STEP convention
/// that a closed edge reuses one vertex.
pub(crate) fn edge_sweep(t0: f64, t1: f64, same_sense: bool, ec_id: u32) -> Result<f64, StepError> {
	use std::f64::consts::TAU;
	if !t0.is_finite() || !t1.is_finite() {
		return Err(StepError::Parse(format!("edge #{ec_id} has non-finite arc endpoint angles")));
	}
	let mut sweep = t1 - t0; // atan2 outputs keep this within [−2π, 2π]
	if same_sense {
		while sweep <= 1e-9 {
			sweep += TAU;
		}
	} else {
		while sweep >= -1e-9 {
			sweep -= TAU;
		}
	}
	Ok(sweep)
}

/// Sample a conic arc from `start` to `end` (kept as the exact vertex positions),
/// sweeping `sweep` radians from parameter `t0` (negative = against the curve's
/// parameterisation). One chord up to `MAX_CHORD_SWEEP`, else the `FULL_TURN_SEGMENTS`
/// pitch.
pub(super) fn sample_arc(start: DVec3, end: DVec3, t0: f64, sweep: f64, eval: impl Fn(f64) -> DVec3) -> Vec<DVec3> {
	use std::f64::consts::TAU;
	let n = if sweep.abs() <= MAX_CHORD_SWEEP {
		1
	} else {
		(sweep.abs() / (TAU / FULL_TURN_SEGMENTS as f64)).ceil() as usize
	};
	let mut pts = Vec::with_capacity(n + 1);
	pts.push(start);
	for k in 1..n {
		pts.push(eval(t0 + sweep * k as f64 / n as f64));
	}
	pts.push(end);
	pts
}

/// Last enumeration argument of an entity (the trailing `.T./.F.` flag).
pub(crate) fn last_enum(e: &Entity) -> Option<String> {
	e.args.iter().rev().find_map(|v| match v {
		Value::Enum(s) => Some(s.clone()),
		_ => None,
	})
}

/// Bit-exact key for de-duplicating coincident vertex positions.
pub(crate) type PosKey = (u64, u64, u64);

/// Bit-exact key for de-duplicating coincident vertex positions.
pub(crate) fn pos_key(p: DVec3) -> PosKey {
	(p.x.to_bits(), p.y.to_bits(), p.z.to_bits())
}

/// `(face id, loop-reversal flag)` pairs of one shell.
pub(crate) type ShellFaces = Vec<(u32, bool)>;

/// Expand a STEP `(distinct knots, multiplicities)` pair into a full knot vector,
/// repeating each distinct knot by its multiplicity.
pub(super) fn expand_knots(distinct: &[f64], mults: &[i64]) -> Vec<f64> {
	let mut k = Vec::new();
	for (&val, &m) in distinct.iter().zip(mults) {
		for _ in 0..m.max(0) {
			k.push(val);
		}
	}
	k
}

/// The argument list of the named sub-record inside a `_COMPLEX` instance's args, if present
/// (e.g. the `RATIONAL_B_SPLINE_CURVE` weights record within a rational B-spline complex).
pub(crate) fn complex_part<'a>(args: &'a [Value], name: &str) -> Option<&'a [Value]> {
	args.iter().find_map(|v| match v {
		Value::Typed(n, a) if n == name => Some(a.as_slice()),
		_ => None,
	})
}

/// Remove consecutive duplicate positions (and a duplicated wrap-around point) from a
/// boundary ring — zero-length segments from degenerate edges in the input.
pub(crate) fn dedup_ring(pts: &mut Vec<DVec3>) {
	pts.dedup_by(|a, b| pos_key(*a) == pos_key(*b));
	while pts.len() > 1 && pos_key(pts[0]) == pos_key(pts[pts.len() - 1]) {
		pts.pop();
	}
}

/// Newell area vector of a polygon (winding-following, UNnormalised — its length is
/// twice the enclosed area, so a periodic slit loop yields a near-zero vector).
pub(crate) fn newell_vector(pts: &[DVec3]) -> DVec3 {
	let mut nv = DVec3::ZERO;
	let len = pts.len();
	for i in 0..len {
		let c = pts[i];
		let d = pts[(i + 1) % len];
		nv.x += (c.y - d.y) * (c.z + d.z);
		nv.y += (c.z - d.z) * (c.x + d.x);
		nv.z += (c.x - d.x) * (c.y + d.y);
	}
	nv
}
