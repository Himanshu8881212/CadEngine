// Copyright (c) LMCAD. Licensed under the MIT License.

//! STEP exporter **size gates** — pins the measured effect of the exporter's
//! size discipline (same-surface face coalescing incl. cone frusta + entity
//! hash-consing, landed 2026-07-09) against the honestly measured
//! PRE-enhancement baseline, and re-proves the non-negotiable safety gate
//! (round-trip volume conservation) on every run.
//!
//! BEFORE constants: measured 2026-07-30 by building the pre-enhancement
//! parent commit (`ee063f6~1` = 618fce5 — plane + full-wrap-cylinder
//! coalescing already present, cone frusta still per-facet, no entity dedup)
//! in a scratch worktree and running the IDENTICAL part + measurement code:
//!
//! ```text
//! BASELINE bytes=1466032 entities=29763 faces=1792
//!   DIRECTION 5130 · CARTESIAN_POINT 5128 · ORIENTED_EDGE 5124
//!   EDGE_CURVE 2562 · LINE 2050 · VECTOR 2050 · AXIS2_PLACEMENT_3D 1540
//!   VERTEX_POINT 1538 · ADVANCED_FACE/EDGE_LOOP/FACE_OUTER_BOUND 1028 each
//!   CONICAL_SURFACE 1024 (per-facet!) · CIRCLE 512 · CYLINDRICAL_SURFACE 2
//! ```
//!
//! CURRENT (first run of this gate, 2026-07-30): bytes=381245 entities=7830 —
//! **3.85× fewer bytes, 3.80× fewer entities**. The histogram is now
//! TOPOLOGY-dominated (ORIENTED_EDGE 3092, EDGE_CURVE/CARTESIAN_POINT 1546,
//! VERTEX_POINT 1538) — topology records are 1:1 with topological
//! vertices/edges per ISO 10303-42 semantics and deliberately never shared;
//! every safely shareable GEOMETRY class (points, directions, placements,
//! vectors, lines, circles, surfaces) is hash-consed: CONICAL_SURFACE 4 (one
//! per frustum, shared by its two half-bands), CIRCLE 6, PLANE 2. The honest
//! remaining headroom is rim-arc chain merging (256 per-chord arc EDGE_CURVEs
//! per rim → 2 per half-rim), which would change exported vertex topology —
//! out of scope here and noted in FRICTION.
//!
//! Round-trip receipt (2026-07-30): source volume 14286.528854610 mm³,
//! re-import 14286.528854047 mm³ — relative error **3.94e-11**, twelve orders
//! under the 0.5% safety gate.

use kernel_brep::math::DVec2;
use kernel_brep::{export_step, import_step, revolve, validate, volume};

/// Pre-enhancement export of [`frustum_stack`] at 256 segments — see the
/// module doc for provenance (measured 2026-07-30 at commit 618fce5).
const BASELINE_BYTES: usize = 1_466_032;
const BASELINE_ENTITIES: usize = 29_763;

/// Ratchet floors for the size reduction (RAISE on improvements, never lower):
/// measured 3.84× / 3.80× on 2026-07-30; the floor keeps ~9% headroom for
/// legitimate drift (e.g. an honest accuracy-record change), while a
/// coalescing fallback to facets (~1.2×) or a dedup regression fails loudly.
const BYTES_REDUCTION_FLOOR: f64 = 3.5;
const ENTITIES_REDUCTION_FLOOR: f64 = 3.5;

/// The revolve-heavy gate part: a 256-segment frustum stack — four apex-free
/// cone bands + one cylinder band + two planar discs. Every band is a full
/// wrap, so the exporter's coalescing (planes, cylinder half-bands, cone
/// half-bands) and the entity dedup all engage.
fn frustum_stack() -> kernel_brep::Solid {
	revolve(
		&[
			DVec2::new(0.0, 0.0),
			DVec2::new(10.0, 0.0),
			DVec2::new(14.0, 6.0),
			DVec2::new(14.0, 10.0),
			DVec2::new(11.0, 16.0),
			DVec2::new(16.0, 22.0),
			DVec2::new(13.0, 26.0),
			DVec2::new(0.0, 26.0),
		],
		256,
	)
}

fn count(hay: &str, needle: &str) -> usize {
	hay.matches(needle).count()
}

/// Total `#N = …;` entity records in the DATA section.
fn entity_count(step: &str) -> usize {
	step.lines().filter(|l| l.trim_start().starts_with('#') && l.contains(" = ")).count()
}

/// `(record name, count)` histogram of the DATA section, descending.
/// Complex (multi-supertype, parenthesised) instances bucket as `(complex)`.
fn entity_histogram(step: &str) -> Vec<(String, usize)> {
	use std::collections::HashMap;
	let mut h: HashMap<String, usize> = HashMap::new();
	for line in step.lines() {
		let Some((_, rest)) = line.split_once(" = ") else { continue };
		let name: String = if rest.starts_with('(') {
			"(complex)".to_string()
		} else {
			rest.chars().take_while(|c| c.is_ascii_alphanumeric() || *c == '_').collect()
		};
		*h.entry(name).or_default() += 1;
	}
	let mut v: Vec<(String, usize)> = h.into_iter().collect();
	v.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(&b.0)));
	v
}

/// The size ratchet: the coalescing + dedup exporter must stay ≥ 3.5× smaller
/// than the recorded pre-enhancement baseline on the revolve-heavy part, in
/// bytes AND entities, with the coalesced structure intact (4 shared
/// CONICAL_SURFACEs, 2 CYLINDRICAL, 2 PLANEs, 12 ADVANCED_FACEs).
#[test]
fn frustum_stack_export_holds_the_measured_size_reduction() {
	let solid = frustum_stack();
	let step = export_step(&solid, "frustum_stack");
	let bytes = step.len();
	let entities = entity_count(&step);
	let bytes_ratio = BASELINE_BYTES as f64 / bytes as f64;
	let entities_ratio = BASELINE_ENTITIES as f64 / entities as f64;

	// REPORTING IS THE PRODUCT: print the current histogram every run so a
	// regression's shape is visible without re-instrumenting.
	println!("frustum_stack STEP: {bytes} bytes, {entities} entities ({bytes_ratio:.2}× / {entities_ratio:.2}× vs pre-enhancement baseline {BASELINE_BYTES} B / {BASELINE_ENTITIES})");
	println!("entity histogram (top 10):");
	for (name, n) in entity_histogram(&step).into_iter().take(10) {
		println!("  {name:<28} {n}");
	}

	let cones = count(&step, "= CONICAL_SURFACE(");
	let cyls = count(&step, "= CYLINDRICAL_SURFACE(");
	let planes = count(&step, "= PLANE(");
	let faces = count(&step, "= ADVANCED_FACE(");
	assert!(
		bytes_ratio >= BYTES_REDUCTION_FLOOR
			&& entities_ratio >= ENTITIES_REDUCTION_FLOOR
			&& cones == 4
			&& cyls == 1
			&& planes == 2
			&& faces == 12,
		"STEP size discipline regressed on the 256-seg frustum stack:\n\
		 bytes {bytes} (baseline {BASELINE_BYTES}, {bytes_ratio:.2}× vs floor {BYTES_REDUCTION_FLOOR:.1}×)\n\
		 entities {entities} (baseline {BASELINE_ENTITIES}, {entities_ratio:.2}× vs floor {ENTITIES_REDUCTION_FLOOR:.1}×)\n\
		 structure: CONICAL_SURFACE {cones} (want 4 — one per frustum, shared by its two half-bands), \
		 CYLINDRICAL_SURFACE {cyls} (want 1 — ditto), PLANE {planes} (want 2), ADVANCED_FACE {faces} (want 12: \
		 2 discs + 2 cylinder + 8 cone half-bands)\n\
		 measured 3.85×/3.80× on 2026-07-30; a fall-back-to-facets or dedup regression lands ~1.2×"
	);
}

/// The non-negotiable safety gate behind the size discipline: the coalesced
/// export must re-import through our own reader as a valid solid with the
/// volume conserved (the exporter itself self-gates per solid at 0.5% and
/// falls back to facets — this test proves the shipped file, not the intent).
#[test]
fn frustum_stack_export_round_trips_volume_conserved() {
	let solid = frustum_stack();
	let v0 = volume(&solid).abs();
	let step = export_step(&solid, "frustum_stack");
	let back = import_step(&step).expect("coalesced frustum-stack STEP must re-import");
	let validity = validate(&back);
	let v1 = volume(&back).abs();
	let rel = (v1 - v0).abs() / v0;
	println!("frustum_stack round-trip: source volume {v0:.9}, re-import {v1:.9}, rel err {rel:.2e}");
	assert!(
		validity.is_valid() && rel < 0.005,
		"round-trip safety gate failed: re-import validity {validity:?}, volume {v1:.6} vs source {v0:.6} (rel err {rel:.2e}, gate 5e-3)"
	);
}
