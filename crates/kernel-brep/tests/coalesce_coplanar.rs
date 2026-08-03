//! FRICTION #20 remedy, measured: a pads-on-plate union carries its planes as
//! 65 fragmented faces on current main; `coalesce_coplanar` must collapse it
//! to the shape's true 16 (plate 6 + 2 pads × 5), conserve volume exactly,
//! stay valid — and keep coplanar ISLANDS (the two pad tops, no shared edge)
//! as separate faces. The §7.4 flush tower is the no-op control: the boolean
//! already emits it clean at 6 faces, and coalescing must leave it at 6.
//!
//! Plus FRICTION #20's RESIDUAL half (closed 2026-07-30): the rebuild used to
//! reset provenance, which made the pass finishing-only. It now carries
//! `FaceName`s through — unmerged faces keep theirs exactly, a merged face
//! inherits the lexicographically-least constituent name — so witness-addressed
//! features re-resolve MID-CHAIN. Gated below by resolving the same stored
//! `EdgeName` + witness to the same edge before and after a coalesce, and by a
//! boolean → coalesce → edge-feature chain that rebuilds bit-identically.

use kernel_brep::math::DVec3;
use kernel_brep::{
	coalesce_coplanar, cuboid, fillet_edge_near, tessellate_default, union, validate, volume, EdgeName, FaceName, FaceSource,
	Solid, Surface,
};

#[test]
fn fragmented_planes_merge_islands_survive_and_clean_input_is_untouched() {
	// no-op control: the flush tower is already clean
	let tower = union(
		&cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(30.0, 20.0, 10.0)),
		&cuboid(DVec3::new(0.0, 0.0, 10.0), DVec3::new(30.0, 20.0, 18.0)),
	);
	let tower_m = coalesce_coplanar(&tower);
	let tower_ok = tower.face_count() == 6
		&& tower_m.face_count() == 6
		&& (volume(&tower_m).abs() - 10800.0).abs() < 1e-6
		&& validate(&tower_m).is_valid();

	// the real case: two pads on a plate — 65 fragmented faces on current main
	let plate = union(
		&union(
			&cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(40.0, 20.0, 5.0)),
			&cuboid(DVec3::new(5.0, 5.0, 5.0), DVec3::new(15.0, 15.0, 9.0)),
		),
		&cuboid(DVec3::new(25.0, 5.0, 5.0), DVec3::new(35.0, 15.0, 9.0)),
	);
	let before = plate.face_count();
	let merged = coalesce_coplanar(&plate);
	// the two pad tops (z = 9 plane) must remain TWO separate island faces
	let pad_tops = merged
		.faces()
		.filter(|&f| match merged.face(f).surface {
			Surface::Plane { origin, normal } => normal.z.abs() > 0.999 && (normal.dot(origin).abs() - 9.0).abs() < 1e-6,
			_ => false,
		})
		.count();
	let v_ok = (volume(&merged).abs() - volume(&plate).abs()).abs() < 1e-6;

	assert!(
		tower_ok
			&& before > 20
			&& merged.face_count() == 16
			&& pad_tops == 2
			&& v_ok
			&& validate(&merged).is_valid()
			&& tessellate_default(&merged).is_watertight(),
		"coalesce contract: tower no-op ok={tower_ok}; plate {before} faces (want the >20 fragmentation this fix \
		 exists for) → {} (want 16), pad-top islands {pad_tops} (want 2), vol ok={v_ok}, validity={:?}, wt={}",
		merged.face_count(),
		validate(&merged),
		tessellate_default(&merged).is_watertight()
	);
}

/// The FRICTION #20 scenario's own geometry: two pads in the MIDDLE of a plate.
/// The boolean leaves the plate's top plane as a field of fragments (measured:
/// 58 edges whose two sides carry the SAME `FaceName` — pure fragmentation
/// seams), and coalescing merges them back into one holed face.
fn fragmenting_pads_on_plate() -> Solid {
	union(
		&union(
			&cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(40.0, 20.0, 5.0)),
			&cuboid(DVec3::new(5.0, 5.0, 5.0), DVec3::new(15.0, 15.0, 9.0)),
		),
		&cuboid(DVec3::new(25.0, 5.0, 5.0), DVec3::new(35.0, 15.0, 9.0)),
	)
}

/// The same shape with the pads FLUSH to the plate's `y = 0` edge, so the
/// merged top face is simply connected (no hole loops). This is the geometry
/// the mid-chain FEATURE gate uses: the fillet's rebuild is single-loop only
/// and now refuses a holed solid loudly (see `fillet.rs`), so a middle-pad
/// plate would exercise that refusal rather than the mid-chain capability.
fn flush_pads_on_plate() -> Solid {
	union(
		&union(
			&cuboid(DVec3::new(0.0, 0.0, 0.0), DVec3::new(40.0, 20.0, 5.0)),
			&cuboid(DVec3::new(5.0, 0.0, 5.0), DVec3::new(15.0, 10.0, 9.0)),
		),
		&cuboid(DVec3::new(25.0, 0.0, 5.0), DVec3::new(35.0, 10.0, 9.0)),
	)
}

/// Edges whose two incident faces carry the SAME [`FaceName`] — pure
/// fragmentation seams: one original plane cut into pieces that a downstream
/// feature then cannot address as one face.
fn same_name_seams(s: &Solid) -> usize {
	s.edges()
		.filter(|&e| {
			let he = *s.half_edge(s.edge(e).half_edge);
			let Some(twin) = he.twin else { return false };
			match (s.face_name(he.face), s.face_name(s.half_edge(twin).face)) {
				(Some(a), Some(b)) => a == b,
				_ => false,
			}
		})
		.count()
}

/// The plate's four VERTICAL corner edges, by persistent name: each is where
/// two of the plate's side walls (A2/A3 = ∓Y, A4/A5 = ∓X) meet.
const CORNER_EDGES: [(u32, u32, [f64; 3]); 4] = [
	(2, 4, [0.0, 0.0, 2.5]),
	(2, 5, [40.0, 0.0, 2.5]),
	(3, 4, [0.0, 20.0, 2.5]),
	(3, 5, [40.0, 20.0, 2.5]),
];

fn corner_name(a: u32, b: u32) -> EdgeName {
	EdgeName::new(
		FaceName { operand: FaceSource::OperandA, source_face: a },
		FaceName { operand: FaceSource::OperandA, source_face: b },
	)
}

#[test]
fn provenance_survives_the_rebuild_so_witnesses_re_resolve_mid_chain() {
	let plate = fragmenting_pads_on_plate();
	let merged = coalesce_coplanar(&plate);

	// (1) Provenance EXISTS after the rebuild at all — the old contract reset it
	// (`from_faces_multiloop` starts fresh), which is precisely what made this a
	// finishing-only pass: every `face_name` came back `None`, so `edge_name`
	// did too and every witness query died at `EdgeNotFound`.
	let named_after = merged.faces().filter(|&f| merged.face_name(f).is_some()).count();

	// (2) Unmerged faces keep their name EXACTLY and merged faces inherit the
	// lexicographically-least constituent, so the surviving name set is a
	// SUBSET of the input's — the rebuild invents no names.
	let names_in: std::collections::BTreeSet<FaceName> = plate.faces().filter_map(|f| plate.face_name(f)).collect();
	let names_out: std::collections::BTreeSet<FaceName> = merged.faces().filter_map(|f| merged.face_name(f)).collect();
	let subset = names_out.is_subset(&names_in);

	// (3) A survived face re-resolves to the SAME geometry through the rebuild:
	// the plate's −Y wall (A2) is one plane before and after; its fragments
	// merge, so the name resolves to FEWER faces covering the SAME total area.
	let wall = FaceName { operand: FaceSource::OperandA, source_face: 2 };
	let area_of = |s: &Solid, name: FaceName| -> (usize, f64) {
		let ids = s.faces_named(name);
		let area: f64 = ids
			.iter()
			.map(|&f| {
				let poly = s.face_polygon(f);
				let mut a = DVec3::ZERO;
				for i in 0..poly.len() {
					a += poly[i].cross(poly[(i + 1) % poly.len()]);
				}
				a.length() * 0.5
			})
			.sum();
		(ids.len(), area)
	};
	let (pre_n, pre_area) = area_of(&plate, wall);
	let (post_n, post_area) = area_of(&merged, wall);
	let same_face = post_n <= pre_n && (post_area - pre_area).abs() < 1e-9 && post_n >= 1;

	// (4) The fragmentation itself is gone: 58 seam edges whose two sides were
	// pieces of ONE named plane (the state that makes a downstream feature say
	// "not a straight edge between two whole planar faces") collapse to zero.
	let (seams_pre, seams_post) = (same_name_seams(&plate), same_name_seams(&merged));

	// (5) Witness-addressed EDGES re-resolve through the rebuild: the plate's
	// four vertical corner edges — "a clean corner nowhere near the cut" in the
	// FRICTION #20 report — resolve by name on both solids, and the witness
	// picks the same segment. Before provenance was carried, `edges_named` came
	// back EMPTY here (every `face_name` was `None`), so a mid-chain feature
	// died at `EdgeNotFound`; that is the half this closes.
	let resolved = |s: &Solid, a: u32, b: u32, w: DVec3| -> Option<(DVec3, DVec3)> {
		let ids = s.edges_named(corner_name(a, b));
		let pick = ids
			.into_iter()
			.min_by(|&x, &y| {
				let seg = |e: kernel_brep::EdgeId| {
					let he = *s.half_edge(s.edge(e).half_edge);
					(s.position(he.origin) + s.position(s.half_edge(he.next).origin)) * 0.5
				};
				(seg(x) - w).length().total_cmp(&(seg(y) - w).length())
			})?;
		let he = *s.half_edge(s.edge(pick).half_edge);
		let (p, q) = (s.position(he.origin), s.position(s.half_edge(he.next).origin));
		Some((p.min(q), p.max(q)))
	};
	let mut same_edge = Vec::new();
	for (a, b, w) in CORNER_EDGES {
		let witness = DVec3::new(w[0], w[1], w[2]);
		let (pre, post) = (resolved(&plate, a, b, witness), resolved(&merged, a, b, witness));
		same_edge.push(pre.is_some() && pre == post);
	}
	let all_same_edge = same_edge.iter().all(|&b| b);

	// (6) The honest residual, pinned rather than hidden: this merged solid
	// carries a face with HOLE loops (the plate top around the two pads), and
	// the fillet's rebuild is single-loop only — so a witness-addressed fillet
	// here REFUSES loudly. It used to return `Ok` with closed=false topology and
	// a NEGATIVE cut; a refusal is the honest state until the rebuild is
	// multi-loop aware. (The mid-chain feature capability itself is gated on
	// hole-free geometry by `a_boolean_coalesce_feature_chain_rebuilds_bit_identically`.)
	let holed_faces = merged.faces().filter(|&f| !merged.face(f).inner.is_empty()).count();
	let refusal = fillet_edge_near(&merged, corner_name(2, 4), 1.0, DVec3::new(0.0, 0.0, 2.5));
	let refused_honestly = refusal.as_ref().err() == Some(&kernel_brep::FilletError::Unsupported);

	assert!(
		named_after == merged.face_count()
			&& subset
			&& same_face
			&& seams_pre > 20
			&& seams_post == 0
			&& all_same_edge
			&& holed_faces == 1
			&& refused_honestly,
		"FRICTION #20 residual — provenance through the coalesce rebuild (was: names reset ⇒ finishing-pass only).\n\
		 named faces after rebuild {named_after}/{} (want all); surviving names ⊆ input names = {subset} \
		 ({} in → {} out);\n\
		 survived face A2 (−Y wall) re-resolves: {pre_n} faces / {pre_area:.6} mm² → {post_n} faces / {post_area:.6} mm² \
		 (same face) = {same_face};\n\
		 same-name fragmentation seams {seams_pre} → {seams_post} (want a large number → 0);\n\
		 the four corner edges re-resolve by name+witness to the SAME segment {same_edge:?} (want all true — \
		 they resolved to NOTHING before provenance was carried);\n\
		 residual: {holed_faces} merged face(s) carry hole loops, so the single-loop fillet rebuild refuses \
		 honestly ({refusal:?}, want Unsupported — it used to hand back closed=false topology with a negative cut)",
		merged.face_count(),
		names_in.len(),
		names_out.len(),
	);
}

#[test]
fn a_boolean_coalesce_feature_chain_rebuilds_bit_identically() {
	// The §16.6-style persistent-feature rebuild with a coalesce in the MIDDLE
	// of the chain — the FRICTION #20 scenario end to end: boolean → coalesce →
	// witness-addressed edge feature. This is the capability the provenance
	// carry unlocks: before it, the fillet after a coalesce died at
	// `EdgeNotFound` because the rebuild had erased every `FaceName`.
	// Two independent evaluations must also agree bit-for-bit — a parametric
	// rebuild is worthless if it is not deterministic.
	let build = || -> (Solid, Result<Solid, kernel_brep::FilletError>) {
		let merged = coalesce_coplanar(&flush_pads_on_plate());
		let (a, b, w) = CORNER_EDGES[0];
		let out = fillet_edge_near(&merged, corner_name(a, b), 1.0, DVec3::new(w[0], w[1], w[2]));
		(merged, out)
	};
	let (merged1, r1) = build();
	let (_, r2) = build();
	let names_resolve = !merged1.edges_named(corner_name(CORNER_EDGES[0].0, CORNER_EDGES[0].1)).is_empty();
	let (Ok(a), Ok(b)) = (&r1, &r2) else {
		panic!(
			"the boolean → coalesce → witness-fillet chain must evaluate mid-chain: edge name resolves after \
			 coalesce = {names_resolve}, fillet = {r1:?}"
		);
	};
	let bits = |s: &Solid| -> Vec<[u64; 3]> {
		(0..s.vertex_count() as u32)
			.map(|i| {
				let p = s.position(kernel_brep::VertexId(i));
				[p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]
			})
			.collect()
	};
	let same_bits = bits(a) == bits(b);
	let same_topo = a.face_count() == b.face_count() && a.edge_count() == b.edge_count() && a.vertex_count() == b.vertex_count();
	let names_a: Vec<Option<FaceName>> = a.faces().map(|f| a.face_name(f)).collect();
	let names_b: Vec<Option<FaceName>> = b.faces().map(|f| b.face_name(f)).collect();
	// The feature did real, correct work: a 1 mm round on the 5 mm-tall corner
	// removes (1 − π/4)·r²·h = 1.0730 mm³; the 16-segment facet fillet is
	// inscribed, so it takes a hair more (measured 1.0793, +0.6%).
	let cut = volume(&merged1) - volume(a);
	let exact = (1.0 - std::f64::consts::FRAC_PI_4) * 5.0;
	let cut_ok = cut > exact * 0.97 && cut < exact * 1.03;
	assert!(
		names_resolve && same_bits && same_topo && names_a == names_b && validate(a).is_valid() && cut_ok,
		"mid-chain rebuild (boolean → coalesce → witness fillet): edge name resolves post-coalesce={names_resolve} \
		 (was impossible — the rebuild reset provenance), bit-identical vertices={same_bits}, same topology={same_topo} \
		 ({}/{}/{} vs {}/{}/{} faces/edges/verts), same provenance={}, validity {:?}, \
		 material removed {cut:.4} mm³ vs (1−π/4)·r²·h = {exact:.4} (bar ±3%) = {cut_ok}",
		a.face_count(),
		a.edge_count(),
		a.vertex_count(),
		b.face_count(),
		b.edge_count(),
		b.vertex_count(),
		names_a == names_b,
		validate(a)
	);
}
