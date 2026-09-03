// Copyright (c) LMCAD. Licensed under the MIT License.

//! The CSG booleans over exact B-reps: `union`, `difference`, `intersection` and
//! the robustness-ordered `union_all` fold.

use std::collections::{BTreeMap, BTreeSet};

use kernel_brep::Solid;

use crate::interp::{err, fetch_solid, EnvValue, Outcome};
use crate::program::OpKind;
use crate::report::{ErrorKind, OpError};

use super::support::{bind_solid};

/// Execute one op of this family. The dispatch table in [`crate::interp`]
/// routes exactly the variants matched below, so the catch-all is dead code
/// kept only to satisfy the compiler.
pub(crate) fn exec(
	op_id: &str,
	env: &mut BTreeMap<String, EnvValue>,
	all_ids: &BTreeSet<String>,
	kind: OpKind,
) -> Result<Outcome, OpError> {
	match kind {
		OpKind::Union { a, b } => {
			let sa = fetch_solid(env, all_ids, op_id, "a", &a)?;
			let sb = fetch_solid(env, all_ids, op_id, "b", &b)?;
			bind_solid(op_id, "union", kernel_brep::union(sa, sb))
		}
		OpKind::Difference { a, b } => {
			let sa = fetch_solid(env, all_ids, op_id, "a", &a)?;
			let sb = fetch_solid(env, all_ids, op_id, "b", &b)?;
			bind_solid(op_id, "difference", kernel_brep::difference(sa, sb))
		}
		OpKind::Intersection { a, b } => {
			let sa = fetch_solid(env, all_ids, op_id, "a", &a)?;
			let sb = fetch_solid(env, all_ids, op_id, "b", &b)?;
			bind_solid(op_id, "intersection", kernel_brep::intersection(sa, sb))
		}
		OpKind::UnionAll { input } => {
			if input.len() < 2 {
				return Err(err(
					ErrorKind::InvalidParam,
					format!("op '{op_id}': union_all needs at least two ids in 'in' (got {})", input.len()),
				));
			}
			// Robustness-aware fold order. A left fold in argument order used to
			// re-arrange the SAME rebuilt face once per contacting operand: four
			// prisms abutting one plate's hole wall died at the third union with
			// `invalid_geometry`, while grouping the mutually-disjoint prisms
			// first and touching the plate once succeeded (friction
			// folding_book_stand F1/F3, 2026-08-27). So operands are folded in
			// ascending order of how many other operands' AABBs they touch —
			// mutually-disjoint operands merge first (a cheap multi-shell
			// union), the touch-everything operand arranges last and once. The
			// result is the same solid (union is associative); ties keep the
			// argument order, so the fold stays deterministic.
			let solids: Vec<&Solid> = input
				.iter()
				.map(|name| fetch_solid(env, all_ids, op_id, "in", name))
				.collect::<Result<_, _>>()?;
			let boxes: Vec<Option<kernel_brep::BoundingBox>> =
				solids.iter().map(|s| kernel_brep::measure::bounding_box(s)).collect();
			let touches = |i: usize, j: usize| -> bool {
				match (&boxes[i], &boxes[j]) {
					(Some(a), Some(b)) => {
						a.min.x <= b.max.x
							&& b.min.x <= a.max.x && a.min.y <= b.max.y
							&& b.min.y <= a.max.y && a.min.z <= b.max.z
							&& b.min.z <= a.max.z
					}
					_ => true, // unknown bounds: treat as touching (conservative)
				}
			};
			let mut order: Vec<usize> = (0..solids.len()).collect();
			let degree: Vec<usize> =
				(0..solids.len()).map(|i| (0..solids.len()).filter(|&j| j != i && touches(i, j)).count()).collect();
			order.sort_by_key(|&i| (degree[i], i));
			let mut acc = kernel_brep::union(solids[order[0]], solids[order[1]]);
			for &i in &order[2..] {
				acc = kernel_brep::union(&acc, solids[i]);
			}
			bind_solid(op_id, "union_all", acc)
		}

		_ => unreachable!("ops::booleans: op routed to the wrong family"),
	}
}
