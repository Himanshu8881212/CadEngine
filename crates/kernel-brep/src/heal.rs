// Copyright (c) LMCAD. Licensed under the MIT License.

//! Tolerant modeling, first slice: heal imports/operands whose shells carry
//! small **gaps and slivers** instead of failing on them (the Parasolid
//! "tolerant modeling" / OCCT "fuzzy boolean" capability class — see BAR.md
//! Level 9 and the contract section in `NUMERICS.md`).
//!
//! ## What [`Solid::heal_tolerant`] does (and reports, loudly)
//! 1. **Weld near-coincident vertices** within the caller's tolerance `tol`
//!    (mm): vertices are clustered against the *first-seen* representative in
//!    vertex-index order via a uniform `tol`-cell spatial hash — deterministic,
//!    no iteration-order dependence — and every face loop is rewritten onto the
//!    representatives. A crack between two faces whose rims drifted apart by
//!    ≤ `tol` (a lossy STEP/STL import, an exploded-and-perturbed shell) closes
//!    because the rewritten loops share vertices again, so the half-edge
//!    twin-matcher pairs the previously open edges.
//! 2. **Collapse and drop degenerates**: consecutive duplicate vertices a weld
//!    produces are collapsed; a loop left with fewer than 3 distinct vertices
//!    is dropped (an inner hole loop silently vanishes *into the report*, an
//!    outer loop drops its whole face); a face whose remaining polygon area is
//!    ≤ `tol²` (point-like at the caller's own tolerance) is dropped as a
//!    sliver.
//! 3. **Re-validate** and return the healed solid plus a [`HealReport`] of
//!    exactly what changed: welded-vertex / dropped-face / dropped-loop counts,
//!    open (unpaired) half-edge counts before and after, and the full
//!    [`Validity`] before and after. Nothing is healed silently.
//!
//! ## What it does NOT do (first slice, stated honestly)
//! - It invents no geometry: a hole bigger than `tol` (a whole missing face)
//!   stays open — the report says so via `open_edges_after`.
//! - T-junction cracks (a vertex of one face lying ON another face's edge
//!   interior) are not healed here; the boolean pipeline heals its own
//!   T-junctions internally (`resolve_t_junctions`), but a heal-only call
//!   leaves them open and reported.
//! - Long slivers (sub-`tol` width but large extent, area > `tol²`) are kept.
//! - Self-intersections and overlapping shells are out of scope.
//!
//! ## Tolerant booleans
//! [`boolean_tolerant`] wires the heal in as an **opt-in pre-pass**: both
//! operands are healed at `tol`, the exact boolean runs unchanged, and the
//! result is *checked* ([`crate::checked`]) — returned only if it validates,
//! else the call fails loudly with the same machine-readable
//! [`BooleanError`] the strict checked API uses. The strict paths
//! ([`union`]/[`try_union`](crate::try_union)/…) are untouched: nothing heals
//! unless the caller asked for it.

use kernel_core::math::DVec3;

use crate::checked::BooleanError;
use crate::geom::Curve;
use crate::mesh_boolean::MeshBoolOp;
use crate::topo::{FaceLoops, Solid};
use crate::validate::{validate, Validity};

/// What [`Solid::heal_tolerant`] changed — the loud, machine-readable record
/// that healing happened (nothing in the tolerant path is repaired silently).
#[derive(Clone, Copy, Debug)]
pub struct HealReport {
	/// Vertices merged into an earlier representative within `tol`.
	pub vertices_welded: usize,
	/// Faces dropped: outer loop collapsed below 3 distinct vertices, or the
	/// remaining polygon area was ≤ `tol²` (a point-like sliver).
	pub faces_dropped: usize,
	/// Inner (hole) loops dropped because they collapsed below 3 distinct vertices.
	pub inner_loops_dropped: usize,
	/// Unpaired (boundary) half-edges before healing — the open crack measure.
	pub open_edges_before: usize,
	/// Unpaired half-edges after healing (0 ⇒ every gap ≤ `tol` was closed).
	pub open_edges_after: usize,
	/// Full validity report of the input.
	pub validity_before: Validity,
	/// Full validity report of the healed result.
	pub validity_after: Validity,
}

impl HealReport {
	/// Whether the heal changed anything at all.
	pub fn healed_anything(&self) -> bool {
		self.vertices_welded > 0 || self.faces_dropped > 0 || self.inner_loops_dropped > 0
	}
}

/// Newell area vector of a polygon (length = 2 · area, direction follows winding).
fn newell_area_vec(poly: &[DVec3]) -> DVec3 {
	let mut n = DVec3::ZERO;
	let len = poly.len();
	for i in 0..len {
		let a = poly[i];
		let b = poly[(i + 1) % len];
		n.x += (a.y - b.y) * (a.z + b.z);
		n.y += (a.z - b.z) * (a.x + b.x);
		n.z += (a.x - b.x) * (a.y + b.y);
	}
	n
}

/// Number of unpaired (twin-less) half-edges — the open-crack measure a heal
/// tries to drive to zero.
fn open_half_edges(s: &Solid) -> usize {
	s.half_edges.iter().filter(|he| he.twin.is_none()).count()
}

impl Solid {
	/// Heal small gaps and slivers at the caller's tolerance `tol` (mm, absolute):
	/// weld near-coincident vertices, close the sliver gaps the weld re-joins,
	/// drop degenerate faces/loops, and re-validate — returning the healed solid
	/// and a loud [`HealReport`] of exactly what changed (see the
	/// [module docs](self) for the precise contract and its honest limits).
	///
	/// Deterministic: clustering scans vertices in index order and keeps the
	/// first-seen representative, so identical input yields the identical healed
	/// solid. `tol = 0` still merges *exactly* coincident duplicate vertices;
	/// the caller's `tol` should comfortably exceed the gap width being closed
	/// (and stay far below the model's feature size — features at or below `tol`
	/// are legitimately collapsed, that is what the tolerance *means*).
	pub fn heal_tolerant(&self, tol: f64) -> (Solid, HealReport) {
		let tol = tol.max(0.0);
		let validity_before = validate(self);
		let open_edges_before = open_half_edges(self);

		// --- 1. Weld: cluster vertices to first-seen representatives within `tol`.
		// A uniform spatial hash with `tol`-sized cells; scanning the 27-cell
		// neighbourhood guarantees any point within `tol` of `p` is found. The scan
		// runs in vertex-index order, so the representative choice (and therefore
		// the whole healed solid) is deterministic.
		use std::collections::HashMap;
		let cell = if tol > 0.0 { tol } else { 1.0 };
		let inv = 1.0 / cell;
		let key = |p: DVec3| ((p.x * inv).round() as i64, (p.y * inv).round() as i64, (p.z * inv).round() as i64);
		let mut grid: HashMap<(i64, i64, i64), Vec<u32>> = HashMap::new();
		let n_in = self.vertex_count();
		// Old vertex index → representative OLD index.
		let mut rep: Vec<u32> = Vec::with_capacity(n_in);
		let mut vertices_welded = 0usize;
		for vi in 0..n_in as u32 {
			let p = self.position(crate::topo::VertexId(vi));
			let k = key(p);
			let mut found: Option<u32> = None;
			'search: for dz in -1..=1 {
				for dy in -1..=1 {
					for dx in -1..=1 {
						if let Some(ids) = grid.get(&(k.0 + dx, k.1 + dy, k.2 + dz)) {
							for &id in ids {
								if (self.position(crate::topo::VertexId(id)) - p).length() <= tol {
									found = Some(id);
									break 'search;
								}
							}
						}
					}
				}
			}
			match found {
				Some(id) => {
					rep.push(id);
					vertices_welded += 1;
				}
				None => {
					rep.push(vi);
					grid.entry(k).or_default().push(vi);
				}
			}
		}

		// --- 2. Rewrite faces onto representatives; collapse and drop degenerates.
		let area_floor = tol * tol;
		let mut faces_dropped = 0usize;
		let mut inner_loops_dropped = 0usize;
		let mut faces: Vec<FaceLoops> = Vec::new();
		let mut kept_provenance = Vec::new();
		for f in self.faces() {
			let face = self.face(f);
			let mut loops: Vec<Vec<u32>> = Vec::new();
			let mut outer_ok = true;
			for (li, lid) in std::iter::once(face.outer).chain(face.inner.iter().copied()).enumerate() {
				// Loop vertex ids, mapped through the weld and stripped of the
				// consecutive (and wrap-around) duplicates the weld created.
				let mut ids: Vec<u32> = self
					.loop_half_edges(lid)
					.into_iter()
					.map(|he| rep[self.half_edge(he).origin.0 as usize])
					.collect();
				ids.dedup();
				while ids.len() > 1 && ids.first() == ids.last() {
					ids.pop();
				}
				if ids.len() < 3 {
					if li == 0 {
						outer_ok = false;
					} else {
						inner_loops_dropped += 1;
					}
					continue;
				}
				if li == 0 {
					// Sliver filter on the outer polygon, at the caller's own scale.
					let poly: Vec<DVec3> = ids.iter().map(|&i| self.position(crate::topo::VertexId(i))).collect();
					if newell_area_vec(&poly).length() * 0.5 <= area_floor {
						outer_ok = false;
						continue;
					}
				}
				loops.push(ids);
			}
			if !outer_ok || loops.is_empty() {
				faces_dropped += 1;
				// Inner loops of a dropped face go with it (not double-counted).
				continue;
			}
			faces.push(FaceLoops { loops, surface: face.surface });
			if let Some(name) = self.face_name(f) {
				kept_provenance.push(name);
			}
		}

		// --- 3. Compact the vertex pool to the representatives actually referenced.
		let mut new_index = vec![u32::MAX; n_in];
		let mut positions: Vec<DVec3> = Vec::new();
		for fl in faces.iter_mut() {
			for lp in fl.loops.iter_mut() {
				for id in lp.iter_mut() {
					let old = *id as usize;
					if new_index[old] == u32::MAX {
						new_index[old] = positions.len() as u32;
						positions.push(self.position(crate::topo::VertexId(*id)));
					}
					*id = new_index[old];
				}
			}
		}

		// Edge-curve tags survive when both endpoints survive: remember them by the
		// welded endpoint pair and re-attach after the rebuild.
		let mut curves: Vec<(u32, u32, Curve)> = Vec::new();
		for e in self.edges() {
			if let Some(c) = self.edge_curve(e) {
				let he = self.edge(e).half_edge;
				let a = rep[self.half_edge(he).origin.0 as usize] as usize;
				let b = rep[self.half_edge(self.half_edge(he).next).origin.0 as usize] as usize;
				let (na, nb) = (new_index[a], new_index[b]);
				if na != u32::MAX && nb != u32::MAX && na != nb {
					curves.push((na, nb, c));
				}
			}
		}

		let mut healed = Solid::from_faces_multiloop(positions, faces);
		if kept_provenance.len() == healed.face_count() && !kept_provenance.is_empty() {
			healed.set_provenance(kept_provenance);
		}
		for (a, b, c) in curves {
			healed.set_edge_curve(crate::topo::VertexId(a), crate::topo::VertexId(b), c);
		}

		let report = HealReport {
			vertices_welded,
			faces_dropped,
			inner_loops_dropped,
			open_edges_before,
			open_edges_after: open_half_edges(&healed),
			validity_before,
			validity_after: validate(&healed),
		};
		(healed, report)
	}
}

/// The result of a [`boolean_tolerant`] call: the validated boolean solid plus
/// the heal reports of both operands, so the caller sees exactly what the
/// tolerant pre-pass changed (possibly nothing) before the boolean ran.
#[derive(Clone, Debug)]
pub struct TolerantBoolean {
	/// The boolean result — validated (closed, manifold, genus ≥ 0).
	pub solid: Solid,
	/// What healing `a` changed.
	pub heal_a: HealReport,
	/// What healing `b` changed.
	pub heal_b: HealReport,
}

/// Exact boolean with an **opt-in tolerant pre-pass**: heal both operands at
/// `tol` ([`Solid::heal_tolerant`]), run the *identical* exact boolean, then
/// validate — returning the result **only if it is a well-formed solid**, else
/// the machine-readable [`BooleanError`] the strict checked API uses. Use this
/// when an operand may carry import-grade gaps/slivers ≤ `tol`; on clean
/// operands the pre-pass is an exact no-op rebuild and the result matches the
/// strict path. The strict spellings ([`crate::union`] / [`crate::try_union`] /
/// …) never heal — gapped input still fails loudly there, by design.
pub fn boolean_tolerant(a: &Solid, b: &Solid, op: MeshBoolOp, tol: f64) -> Result<TolerantBoolean, BooleanError> {
	let (ha, heal_a) = a.heal_tolerant(tol);
	let (hb, heal_b) = b.heal_tolerant(tol);
	let (raw, name) = match op {
		MeshBoolOp::Union => (crate::booleans::union(&ha, &hb), "union"),
		MeshBoolOp::Difference => (crate::booleans::difference(&ha, &hb), "difference"),
		MeshBoolOp::Intersection => (crate::booleans::intersection(&ha, &hb), "intersection"),
	};
	let validity = validate(&raw);
	if validity.is_valid() {
		Ok(TolerantBoolean { solid: raw, heal_a, heal_b })
	} else {
		Err(BooleanError { op: name, validity })
	}
}

#[cfg(test)]
mod tests {
	use super::*;
	use crate::build::{cuboid, cylinder};
	use crate::checked::try_union;
	use crate::topo::FaceInput;
	use crate::validate::{validate, volume};
	use kernel_core::math::DVec3;

	/// Deterministic xorshift64 in [0, 1).
	struct Rng(u64);
	impl Rng {
		fn next_f64(&mut self) -> f64 {
			self.0 ^= self.0 << 13;
			self.0 ^= self.0 >> 7;
			self.0 ^= self.0 << 17;
			(self.0 >> 11) as f64 / (1u64 << 53) as f64
		}
	}

	/// Explode `s` so every face owns private copies of its vertices, each copy
	/// perturbed by a random direction with magnitude in `[lo, hi]` — a
	/// deliberately cracked shell of the kind a lossy import produces. Every
	/// face is present; only the shared-vertex identification is broken.
	fn cracked(s: &Solid, lo: f64, hi: f64, seed: u64) -> Solid {
		let mut rng = Rng(seed);
		let mut positions: Vec<DVec3> = Vec::new();
		let mut faces: Vec<FaceInput> = Vec::new();
		for f in s.faces() {
			let poly = s.face_polygon(f);
			let base = positions.len() as u32;
			for p in &poly {
				let dir = DVec3::new(
					rng.next_f64() * 2.0 - 1.0,
					rng.next_f64() * 2.0 - 1.0,
					rng.next_f64() * 2.0 - 1.0,
				)
				.normalize_or_zero();
				let mag = lo + (hi - lo) * rng.next_f64();
				positions.push(*p + dir * mag);
			}
			faces.push(FaceInput {
				boundary: (base..base + poly.len() as u32).collect(),
				surface: s.face(f).surface,
			});
		}
		Solid::from_faces(positions, faces)
	}

	#[test]
	fn cracked_shell_heals_to_a_valid_solid_with_a_loud_report() {
		// A 10 mm cube exploded into 6 quads with private vertices, every copy
		// perturbed by 1e-5..1e-4 mm: all 24 half-edge pairs are torn open (gaps far
		// above the boolean's WELD_EPS = 1e-7), so the strict world calls it broken.
		let cube = cuboid(DVec3::ZERO, DVec3::splat(10.0));
		let crk = cracked(&cube, 1e-5, 1e-4, 0x5eed_cafe_f00d_0001);
		let before = validate(&crk);
		let (healed, r) = crk.heal_tolerant(1e-3);
		let after = validate(&healed);
		let vol = volume(&healed);
		assert!(
			!before.closed
				&& r.open_edges_before == 24
				&& r.open_edges_after == 0
				&& r.vertices_welded == 16 // 24 copies cluster back to 8 corners
				&& r.faces_dropped == 0
				&& r.inner_loops_dropped == 0
				&& r.healed_anything()
				&& after.is_valid()
				&& after.genus == 0
				&& healed.vertex_count() == 8
				&& healed.face_count() == 6
				&& (vol - 1000.0).abs() < 1.0e-2, // perturbations ≤ 1e-4 on a 10 mm cube
			"cracked cube must heal to a valid 6-face/8-vertex cube: before={before:?} report={r:?} after={after:?} vol={vol}"
		);
	}

	#[test]
	fn heal_is_a_no_op_on_a_clean_solid_and_keeps_curved_tags() {
		// Healing must not invent changes: a clean drilled plate (planar + cylinder
		// faces, an edge with topology already perfect) rebuilds identically — zero
		// welds/drops, volume bit-identical, curved surface tags still present.
		let plate = cuboid(DVec3::new(-10.0, -10.0, -3.0), DVec3::new(10.0, 10.0, 3.0));
		let bore = cylinder(DVec3::new(0.0, 0.0, -4.0), DVec3::Z, 2.5, 8.0, 32);
		let drilled = crate::booleans::difference(&plate, &bore);
		let (healed, r) = drilled.heal_tolerant(1e-6);
		let curved = |s: &Solid| s.faces().filter(|&f| !matches!(s.face(f).surface, crate::geom::Surface::Plane { .. })).count();
		assert!(
			!r.healed_anything()
				&& r.open_edges_before == 0
				&& r.open_edges_after == 0
				&& r.validity_after.is_valid()
				&& healed.face_count() == drilled.face_count()
				&& curved(&healed) == curved(&drilled)
				&& volume(&healed).to_bits() == volume(&drilled).to_bits(),
			"clean solid must heal as an exact no-op: report={r:?}, faces {} -> {}, curved {} -> {}, vol {} vs {}",
			drilled.face_count(),
			healed.face_count(),
			curved(&drilled),
			curved(&healed),
			volume(&drilled),
			volume(&healed)
		);
	}

	#[test]
	fn heal_is_deterministic() {
		// Same cracked input twice ⇒ bit-identical healed solid (index-order weld,
		// no HashMap iteration order anywhere in the path).
		let cube = cuboid(DVec3::ZERO, DVec3::splat(7.0));
		let crk = cracked(&cube, 1e-5, 1e-4, 0xd00d_beef_0bad_cafe);
		let (h1, r1) = crk.heal_tolerant(5e-4);
		let (h2, r2) = crk.heal_tolerant(5e-4);
		let snap = |s: &Solid| {
			(
				s.vertex_count(),
				s.face_count(),
				s.edge_count(),
				volume(s).to_bits(),
				(0..s.vertex_count() as u32)
					.map(|i| s.position(crate::topo::VertexId(i)).to_array().map(f64::to_bits))
					.collect::<Vec<_>>(),
			)
		};
		assert!(
			snap(&h1) == snap(&h2) && r1.vertices_welded == r2.vertices_welded && r1.open_edges_after == r2.open_edges_after,
			"heal must be deterministic: {:?} vs {:?}",
			snap(&h1).0..=snap(&h1).2,
			snap(&h2).0..=snap(&h2).2
		);
	}

	#[test]
	fn gapped_boolean_fails_strict_and_succeeds_tolerant() {
		// THE tolerant-modeling acceptance: the SAME gapped operand pair must
		// (a) fail loudly through the strict checked path, and (b) succeed through
		// the tolerant path — both behaviors asserted, neither silently degraded.
		let a = cracked(&cuboid(DVec3::ZERO, DVec3::splat(10.0)), 2e-5, 1e-4, 0xbad5_eed5_0000_0042);
		let b = cuboid(DVec3::new(5.0, 5.0, 5.0), DVec3::new(15.0, 15.0, 15.0));
		assert!(!validate(&a).closed, "fixture must be a genuinely open (cracked) shell");

		// Strict path: refuses, with the broken invariants named.
		let strict = try_union(&a, &b).expect_err("strict checked union of a cracked shell must fail loudly");
		// Tolerant path: heals the cracks at 1e-3, then the identical boolean succeeds.
		let tol = boolean_tolerant(&a, &b, MeshBoolOp::Union, 1e-3).expect("tolerant union must succeed on ≤1e-4 gaps");
		let v = validate(&tol.solid);
		let vol = volume(&tol.solid);
		// Inclusion–exclusion for the two 10-cubes overlapping in a 5-cube:
		// 1000 + 1000 − 125 = 1875, up to the ≤1e-4 crack perturbation.
		assert!(
			!strict.validity.closed
				&& v.is_valid()
				&& tol.heal_a.open_edges_before > 0
				&& tol.heal_a.open_edges_after == 0
				&& tol.heal_a.vertices_welded == 16
				&& !tol.heal_b.healed_anything()
				&& (vol - 1875.0).abs() < 5e-2,
			"strict must fail / tolerant must succeed: strict={strict} heal_a={:?} heal_b={:?} v={v:?} vol={vol}",
			tol.heal_a,
			tol.heal_b
		);
	}

	#[test]
	fn tolerant_boolean_on_clean_operands_matches_the_strict_result() {
		// The pre-pass must be an exact no-op on clean input: tolerant and strict
		// agree to the BIT (the boolean pipeline is deterministic, R5).
		let a = cuboid(DVec3::ZERO, DVec3::splat(10.0));
		let b = cylinder(DVec3::new(5.0, 5.0, -2.0), DVec3::Z, 3.0, 14.0, 24);
		let strict = try_union(&a, &b).expect("clean union validates");
		let tol = boolean_tolerant(&a, &b, MeshBoolOp::Union, 1e-3).expect("tolerant union validates");
		assert!(
			volume(&strict).to_bits() == volume(&tol.solid).to_bits()
				&& strict.face_count() == tol.solid.face_count()
				&& !tol.heal_a.healed_anything()
				&& !tol.heal_b.healed_anything(),
			"tolerant == strict on clean operands: vol {} vs {}, faces {} vs {}",
			volume(&strict),
			volume(&tol.solid),
			strict.face_count(),
			tol.solid.face_count()
		);
	}

	#[test]
	fn sliver_strip_collapses_under_the_weld_and_is_dropped() {
		// A 10-cube whose top-front edge is duplicated 1e-5 apart, with a 1e-5-wide
		// sliver QUAD bridging the two copies (the classic import artifact: a
		// "crack filler" strip). The solid is CLOSED — the strip is real topology —
		// but at tol = 1e-3 the weld merges the edge copies, the strip collapses to
		// 2 distinct vertices, and the heal drops it: a perfect 6-face cube remains.
		let p = |x: f64, y: f64, z: f64| DVec3::new(x, y, z);
		let positions = vec![
			p(0.0, 0.0, 0.0),
			p(10.0, 0.0, 0.0),
			p(10.0, 10.0, 0.0),
			p(0.0, 10.0, 0.0),
			p(0.0, 0.0, 10.0),  // 4: front-top edge, front copy
			p(10.0, 0.0, 10.0), // 5
			p(10.0, 10.0, 10.0),
			p(0.0, 10.0, 10.0),
			p(0.0, 1e-5, 10.0),  // 8 = 4', the top-face copy of 4
			p(10.0, 1e-5, 10.0), // 9 = 5'
		];
		let plane = |normal: DVec3| crate::geom::Surface::Plane { origin: DVec3::ZERO, normal };
		let faces = vec![
			FaceInput { boundary: vec![0, 3, 2, 1], surface: plane(-DVec3::Z) },
			FaceInput { boundary: vec![0, 1, 5, 4], surface: plane(-DVec3::Y) },
			FaceInput { boundary: vec![1, 2, 6, 9, 5], surface: plane(DVec3::X) },
			FaceInput { boundary: vec![2, 3, 7, 6], surface: plane(DVec3::Y) },
			FaceInput { boundary: vec![3, 0, 4, 8, 7], surface: plane(-DVec3::X) },
			FaceInput { boundary: vec![8, 9, 6, 7], surface: plane(DVec3::Z) },
			FaceInput { boundary: vec![4, 5, 9, 8], surface: plane(DVec3::Z) }, // the strip
		];
		let s = Solid::from_faces(positions, faces);
		let before = validate(&s);
		let (healed, r) = s.heal_tolerant(1e-3);
		let after = validate(&healed);
		let vol = volume(&healed);
		assert!(
			before.is_valid() // the strip is real, closed topology before healing
				&& r.vertices_welded == 2 // 8→4, 9→5
				&& r.faces_dropped == 1 // the collapsed strip
				&& r.open_edges_before == 0
				&& r.open_edges_after == 0
				&& after.is_valid()
				&& healed.face_count() == 6
				&& healed.vertex_count() == 8
				&& (vol - 1000.0).abs() < 2e-3, // the 10×1e-5 strip's sliver of volume
			"the sliver strip must collapse and drop, leaving a clean closed cube: before={before:?} report={r:?} after={after:?} vol={vol}"
		);
	}

	#[test]
	fn point_like_face_is_dropped_by_the_area_floor() {
		// A needle face whose vertices are too far apart to weld (base 1 mm) but
		// whose area (½·1·1e-7 = 5e-8 mm²) is below the tol² = 1e-6 floor, next to
		// an honest quad: the needle drops by AREA, the quad survives. (An open
		// sheet — heal operates on any face set, not only closed shells.)
		let p = |x: f64, y: f64, z: f64| DVec3::new(x, y, z);
		let positions = vec![
			p(0.0, 0.0, 0.0),
			p(1.0, 0.0, 0.0),
			p(0.5, 1e-7, 0.0), // needle apex: area 5e-8, no vertex within tol of another
			p(5.0, 0.0, 0.0),
			p(6.0, 0.0, 0.0),
			p(6.0, 1.0, 0.0),
			p(5.0, 1.0, 0.0),
		];
		let plane = crate::geom::Surface::Plane { origin: DVec3::ZERO, normal: DVec3::Z };
		let faces = vec![
			FaceInput { boundary: vec![0, 1, 2], surface: plane },
			FaceInput { boundary: vec![3, 4, 5, 6], surface: plane },
		];
		let s = Solid::from_faces(positions, faces);
		let (healed, r) = s.heal_tolerant(1e-3);
		assert!(
			r.vertices_welded == 0 && r.faces_dropped == 1 && healed.face_count() == 1 && healed.vertex_count() == 4,
			"the sub-tol² needle must drop by area, the quad must survive: report={r:?}, faces={} verts={}",
			healed.face_count(),
			healed.vertex_count()
		);
	}
}
